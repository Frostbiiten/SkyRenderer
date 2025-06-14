#include "PixelBuff.h"
#include "cassert"
#include <algorithm>
#include <raylib-cpp.hpp>
#include <string_view>
#include "Model.h"
#include <limits>
#include <vector>
#include <thread>
#include <functional>
#include <numeric>
#include <execution>
#include <mutex>
#include <cstring>

#define PIXELBUFF_ENABLE_SIMD_FILL4
#define PIXELBUFF_ENABLE_SIMD

namespace sky
{
    static inline float dot4(simde__m128 a, simde__m128 b)
    {
        simde__m128 mul = simde_mm_mul_ps(a, b);
        simde__m128 shuf1 = simde_mm_shuffle_ps(mul, mul, SIMDE_MM_SHUFFLE(2, 3, 0, 1));
        simde__m128 sum1 = simde_mm_add_ps(mul, shuf1);
        simde__m128 shuf2 = simde_mm_shuffle_ps(sum1, sum1, SIMDE_MM_SHUFFLE(1, 0, 3, 2));
        simde__m128 sum2 = simde_mm_add_ps(sum1, shuf2);
        return simde_mm_cvtss_f32(sum2);
    }

    static inline Vector4 Vector4Transform(const Vector4& v, const Matrix& m)
    {
        simde__m128 vec = simde_mm_set_ps(v.w, v.z, v.y, v.x); // reversed order

        simde__m128 row0 = simde_mm_set_ps(m.m12, m.m8, m.m4, m.m0);
        simde__m128 row1 = simde_mm_set_ps(m.m13, m.m9, m.m5, m.m1);
        simde__m128 row2 = simde_mm_set_ps(m.m14, m.m10, m.m6, m.m2);
        simde__m128 row3 = simde_mm_set_ps(m.m15, m.m11, m.m7, m.m3);

        float x = dot4(row0, vec);
        float y = dot4(row1, vec);
        float z = dot4(row2, vec);
        float w = dot4(row3, vec);

        return { x, y, z, w };
    }

    static inline VertexAttributes lerp_attributes(const VertexAttributes& a, const VertexAttributes& b, float t)
    {
        return {
                Vector3Lerp(a.world_normal, b.world_normal, t),
                Vector3Lerp(a.world_pos, b.world_pos, t)
        };
    }

    Camera::Camera(float fov, Vector3 position, Quaternion rotation)
            :
            position { position },
            rotation { rotation },
            fov(fov)
    {
    }

    Matrix Camera::get_view_matrix() const
    {
        Matrix invRot = MatrixInvert(QuaternionToMatrix(rotation));
        Matrix invPos = MatrixTranslate(-position.x, -position.y, -position.z);
        return MatrixMultiply(invPos, invRot);
    }

    float Camera::get_near_clip() const
    {
        return near_clip;
    }

    float Camera::get_far_clip() const
    {
        return far_clip;
    }

    Matrix Camera::get_projection_matrix(int screenWidth, int screenHeight) const
    {
        return MatrixPerspective(fov * DEG2RAD, (float)screenWidth / screenHeight, near_clip, far_clip);
    }

    // [0, 1] mapped to [0, 2^24-1]
    constexpr uint32_t DEPTH_SCALE = (1 << 24) - 1;
    inline uint32_t float_to_fixed_depth(float z)
    {
        z = std::clamp(z, 0.0f, 1.0f);
        return static_cast<uint32_t>(z * DEPTH_SCALE);
    }

    inline float fixed_to_float_depth(uint32_t d)
    {
        return float(d) / float(DEPTH_SCALE);
    }

    PixelBuff::PixelBuff(int WIDTH, int HEIGHT)
            : WIDTH(WIDTH), HEIGHT(HEIGHT),
            framebuffer(WIDTH* HEIGHT, 0),
            cocMap(WIDTH* HEIGHT, 0),
            colourLin(WIDTH* HEIGHT, Vector3{}),
            depthbuffer(WIDTH* HEIGHT, DEPTH_SCALE)
    {
    };

    void PixelBuff::set(int x, int y, std::uint32_t color)
    {
        framebuffer[y * WIDTH + x] = color;
    }

    bool Vector3NearEqual(const Vector3& a, const Vector3& b, float eps)
    {
        return std::abs(a.x - b.x) < eps &&
               std::abs(a.y - b.y) < eps &&
               std::abs(a.z - b.z) < eps;
    }

    bool is_flat_shaded(const VertexAttributes& a1,
                        const VertexAttributes& a2,
                        const VertexAttributes& a3)
    {
        constexpr float epsilon = 1e-5f;
        return  Vector3NearEqual(a1.world_normal, a2.world_normal, epsilon)
                && Vector3NearEqual(a1.world_normal, a3.world_normal, epsilon)
                && Vector3NearEqual(a1.world_pos,    a2.world_pos,    epsilon)
                && Vector3NearEqual(a1.world_pos,    a3.world_pos,    epsilon);
    }

    void PixelBuff::draw_ss_triangle(const Vector3& p1, const Vector3& p2, const Vector3& p3, const VertexAttributes& a1, const VertexAttributes& a2, const VertexAttributes& a3, const Camera& camera, ShadeFunc shader)
    {
        float area = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        if (area >= 0.0f) return;

        int min_x = std::max((int)std::floor(std::min({ p1.x, p2.x, p3.x })), 0);
        int max_x = std::min((int)std::ceil(std::max({ p1.x, p2.x, p3.x })), WIDTH - 1);
        int min_y = std::max((int)std::floor(std::min({ p1.y, p2.y, p3.y })), 0);
        int max_y = std::min((int)std::ceil(std::max({ p1.y, p2.y, p3.y })), HEIGHT - 1);

        float x_step0 = p2.y - p3.y; float y_step0 = p3.x - p2.x;
        float x_step1 = p3.y - p1.y; float y_step1 = p1.x - p3.x;
        float x_step2 = p1.y - p2.y; float y_step2 = p2.x - p1.x;

        Vector3 p_start = { (float)min_x + 0.5f, (float)min_y + 0.5f, 0.0f };
        float w0_row = (p3.x - p2.x) * (p_start.y - p2.y) - (p3.y - p2.y) * (p_start.x - p2.x);
        float w1_row = (p1.x - p3.x) * (p_start.y - p3.y) - (p1.y - p3.y) * (p_start.x - p3.x);
        float w2_row = (p2.x - p1.x) * (p_start.y - p1.y) - (p2.y - p1.y) * (p_start.x - p1.x);

        float inv_area = 1.0f / area;

        for (int y = min_y; y <= max_y; ++y)
        {
            float w0 = w0_row;
            float w1 = w1_row;
            float w2 = w2_row;

            simde__m128 pz1 = simde_mm_set1_ps(p1.z);
            simde__m128 pz2 = simde_mm_set1_ps(p2.z);
            simde__m128 pz3 = simde_mm_set1_ps(p3.z);

#ifdef PIXELBUFF_ENABLE_SIMD_FILL4
            // Barycentric edge functions for 4 pixels
            simde__m128 dx0 = simde_mm_set1_ps(p3.x - p2.x);
            simde__m128 dy0 = simde_mm_set1_ps(p3.y - p2.y);
            simde__m128 dx1 = simde_mm_set1_ps(p1.x - p3.x);
            simde__m128 dy1 = simde_mm_set1_ps(p1.y - p3.y);
            simde__m128 dx2 = simde_mm_set1_ps(p2.x - p1.x);
            simde__m128 dy2 = simde_mm_set1_ps(p2.y - p1.y);

            for (int x = min_x; x <= max_x; x += 4)
            {
                simde__m128i xi = simde_mm_set_epi32(x + 3, x + 2, x + 1, x + 0);
                simde__m128 xf = simde_mm_cvtepi32_ps(xi);
                simde__m128 yf = simde_mm_set1_ps((float)y + 0.5f);

                simde__m128 px = simde_mm_add_ps(xf, simde_mm_set1_ps(0.5f));  // center of pixels

                /* Moved outside ...
                simde__m128 dx0 = simde_mm_set1_ps(p3.x - p2.x);
                simde__m128 dy0 = simde_mm_set1_ps(p3.y - p2.y);
                simde__m128 dx1 = simde_mm_set1_ps(p1.x - p3.x);
                simde__m128 dy1 = simde_mm_set1_ps(p1.y - p3.y);
                simde__m128 dx2 = simde_mm_set1_ps(p2.x - p1.x);
                simde__m128 dy2 = simde_mm_set1_ps(p2.y - p1.y);
                 */

                simde__m128 w0 = simde_mm_sub_ps(simde_mm_mul_ps(dx0, simde_mm_sub_ps(yf, simde_mm_set1_ps(p2.y))),
                                                 simde_mm_mul_ps(dy0, simde_mm_sub_ps(px, simde_mm_set1_ps(p2.x))));
                simde__m128 w1 = simde_mm_sub_ps(simde_mm_mul_ps(dx1, simde_mm_sub_ps(yf, simde_mm_set1_ps(p3.y))),
                                                 simde_mm_mul_ps(dy1, simde_mm_sub_ps(px, simde_mm_set1_ps(p3.x))));
                simde__m128 w2 = simde_mm_sub_ps(simde_mm_mul_ps(dx2, simde_mm_sub_ps(yf, simde_mm_set1_ps(p1.y))),
                                                 simde_mm_mul_ps(dy2, simde_mm_sub_ps(px, simde_mm_set1_ps(p1.x))));

                // inside test (w0 <= 0 && w1 <= 0 && w2 <= 0)
                simde__m128 mask = simde_mm_and_ps(
                        simde_mm_cmple_ps(w0, simde_mm_setzero_ps()),
                        simde_mm_and_ps(simde_mm_cmple_ps(w1, simde_mm_setzero_ps()),
                                        simde_mm_cmple_ps(w2, simde_mm_setzero_ps()))
                );

                if (simde_mm_movemask_ps(mask) == 0) continue; // no pixel covered

                // Compute barycentrics
                simde__m128 inv_area_ps = simde_mm_set1_ps(inv_area);
                simde__m128 b0 = simde_mm_mul_ps(w0, inv_area_ps);
                simde__m128 b1 = simde_mm_mul_ps(w1, inv_area_ps);
                simde__m128 b2 = simde_mm_mul_ps(w2, inv_area_ps);

                // Compute z = b0*p1.z + b1*p2.z + b2*p3.z
                /* Moved outside ...
                simde__m128 pz1 = simde_mm_set1_ps(p1.z);
                simde__m128 pz2 = simde_mm_set1_ps(p2.z);
                simde__m128 pz3 = simde_mm_set1_ps(p3.z);
                 */

                simde__m128 z = simde_mm_add_ps(
                        simde_mm_add_ps(simde_mm_mul_ps(b0, pz1),
                                        simde_mm_mul_ps(b1, pz2)),
                        simde_mm_mul_ps(b2, pz3));

                alignas(16) float z_arr[4];
                alignas(16) int   x_arr[4];
                simde_mm_store_ps(z_arr, z);
                simde_mm_storeu_si128(reinterpret_cast<simde__m128i*>(x_arr), xi);

                for (int i = 0; i < 4; ++i)
                {
                    if (!(simde_mm_movemask_ps(mask) & (1 << i))) continue;

                    int px = x_arr[i];
                    int idx = y * WIDTH + px;
                    std::uint32_t zval = float_to_fixed_depth(z_arr[i]);
                    if (px >= WIDTH || idx >= WIDTH * HEIGHT || zval >= depthbuffer[idx]) continue;

                    depthbuffer[idx] = zval;

                    // Scalar fallback for attributes + shading
                    float b0s = ((float*)&b0)[i];
                    float b1s = ((float*)&b1)[i];
                    float b2s = ((float*)&b2)[i];

                    Vector3 world_pos = barycentric_normalized_vec3(
                            a1.world_pos, a2.world_pos, a3.world_pos,
                            b0s, b1s, b2s);

                    /*
                    Vector3 world_pos = Vector3Add(Vector3Add(
                        Vector3Scale(a1.world_pos, b0s),
                        Vector3Scale(a2.world_pos, b1s)),
                                                   Vector3Scale(a3.world_pos, b2s));

                    Vector3 normal = Vector3NormalizeFast(Vector3Add(Vector3Add(
                        Vector3Scale(a1.world_normal, b0s),
                        Vector3Scale(a2.world_normal, b1s)),
                        Vector3Scale(a3.world_normal, b2s)));
                    */
                    Vector3 normal = barycentric_normalized_vec3(
                            a1.world_normal, a2.world_normal, a3.world_normal,
                            b0s, b1s, b2s);
                    Vector3 viewDir = Vector3NormalizeFast(Vector3Subtract(camera.position, world_pos));
                    set(px, y, shader(normal, viewDir, kDefaultLightDir));
                }
            }
#else
            for (int x = min_x; x <= max_x; ++x)
            {
                if (w0 <= 0 && w1 <= 0 && w2 <= 0)
                {
                    float b0 = w0 * inv_area;
                    float b1 = w1 * inv_area;
                    float b2 = w2 * inv_area;
                    std::uint32_t z = float_to_fixed_depth(b0 * p1.z + b1 * p2.z + b2 * p3.z);
                    int idx = y * WIDTH + x;

                    if (z < depthbuffer[idx])
                    {
                        depthbuffer[idx] = z;

                    #ifdef PIXELBUFF_ENABLE_SIMD
                        simde__m128 bc = simde_mm_set_ps(0.0f, b2, b1, b0);

                        auto pack_vec3 = [](const Vector3& v) -> simde__m128
                        {
                            return simde_mm_set_ps(0.0f, v.z, v.y, v.x);
                        };

                        simde__m128 n0 = pack_vec3(a1.world_normal);
                        simde__m128 n1 = pack_vec3(a2.world_normal);
                        simde__m128 n2 = pack_vec3(a3.world_normal);

                        simde__m128 pos0 = pack_vec3(a1.world_pos);
                        simde__m128 pos1 = pack_vec3(a2.world_pos);
                        simde__m128 pos2 = pack_vec3(a3.world_pos);

                        simde__m128 interp_n = simde_mm_add_ps(
                            simde_mm_add_ps(
                                simde_mm_mul_ps(n0, simde_mm_shuffle_ps(bc, bc, 0x00)),
                                simde_mm_mul_ps(n1, simde_mm_shuffle_ps(bc, bc, 0x55))),
                            simde_mm_mul_ps(n2, simde_mm_shuffle_ps(bc, bc, 0xAA))
                        );

                        simde__m128 interp_p = simde_mm_add_ps(
                            simde_mm_add_ps(
                                simde_mm_mul_ps(pos0, simde_mm_shuffle_ps(bc, bc, 0x00)),
                                simde_mm_mul_ps(pos1, simde_mm_shuffle_ps(bc, bc, 0x55))),
                            simde_mm_mul_ps(pos2, simde_mm_shuffle_ps(bc, bc, 0xAA))
                        );

                        auto unpack_vec3 = [](simde__m128 v) -> Vector3
                        {
                            return {
                                simde_mm_cvtss_f32(v),
                                simde_mm_cvtss_f32(simde_mm_shuffle_ps(v, v, 0x55)),
                                simde_mm_cvtss_f32(simde_mm_shuffle_ps(v, v, 0xAA))
                            };
                        };

                        Vector3 normal = Vector3Normalize(unpack_vec3(interp_n));
                        Vector3 world_pos = unpack_vec3(interp_p);
                    #else
                        Vector3 world_pos = Vector3Add(Vector3Add(
                            Vector3Scale(a1.world_pos, b0),
                            Vector3Scale(a2.world_pos, b1)),
                                                       Vector3Scale(a3.world_pos, b2));

                        Vector3 normal = Vector3Normalize(Vector3Add(Vector3Add(
                            Vector3Scale(a1.world_normal, b0),
                            Vector3Scale(a2.world_normal, b1)),
                            Vector3Scale(a3.world_normal, b2)));
                    #endif
                        Vector3 viewDir = Vector3Normalize(Vector3Subtract(camera.position, world_pos));
                        uint32_t color = shader(normal, viewDir, kDefaultLightDir);
                        set(x, y, color);
                    }
                }

                w0 += x_step0;
                w1 += x_step1;
                w2 += x_step2;
            }
#endif

            w0_row += y_step0;
            w1_row += y_step1;
            w2_row += y_step2;
        }
    }

    void PixelBuff::draw_model(const Camera& camera, const Model& model, ShadeFunc shader)
    {
        const auto& faces = model.get_faces();
        const auto& verts = model.get_vertices();
        const auto& normals = model.get_normals();

        Matrix view = camera.get_view_matrix();
        Matrix proj = camera.get_projection_matrix(WIDTH, HEIGHT);
        Matrix modelMx = model.get_transform();
        Matrix normalMx = MatrixTranspose(MatrixInvert(modelMx));

        std::vector<Vector3> modelTransformedVerts(verts.size());
        for (size_t i = 0; i < verts.size(); ++i)
        {
            modelTransformedVerts[i] = Vector3Transform(verts[i], modelMx);
        }

        std::vector<Vector3> modelTransformedNormals(normals.size());
        for (size_t i = 0; i < normals.size(); ++i)
        {
            Vector4 n = { normals[i].x, normals[i].y, normals[i].z, 0.0f };
            Vector4 tn = Vector4Transform(n, normalMx);
            modelTransformedNormals[i] = Vector3Normalize({ tn.x, tn.y, tn.z });
        }

        std::vector<Vector3> camVerts(verts.size());
        for (size_t i = 0; i < verts.size(); ++i)
        {
            camVerts[i] = Vector3Transform(modelTransformedVerts[i], view);
        }

        auto cam_to_screen = [&](const Vector3& cam) -> Vector3
        {
            Vector4 clip = Vector4Transform({ cam.x, cam.y, cam.z, 1.0f }, proj);
            if (clip.w == 0.0f) return { 0,0, std::numeric_limits<float>::infinity() };
            float rcp_w = 1.0f / clip.w;
            float ndcX = clip.x * rcp_w;
            float ndcY = clip.y * rcp_w;
            float ndcZ = clip.z * rcp_w;
            return {
                    (ndcX * 0.5f + 0.5f) * WIDTH,
                    (1.0f - (ndcY * 0.5f + 0.5f)) * HEIGHT,
                    ndcZ
            };
        };

        for (const auto& cur : faces)
        {
            std::array<VertexAttributes, 3> faceAttributes = {
                    VertexAttributes{modelTransformedNormals[cur.vertices[0].vn], modelTransformedVerts[cur.vertices[0].v]},
                    VertexAttributes{modelTransformedNormals[cur.vertices[1].vn], modelTransformedVerts[cur.vertices[1].v]},
                    VertexAttributes{modelTransformedNormals[cur.vertices[2].vn], modelTransformedVerts[cur.vertices[2].v]}
            };

            std::array<Vector3, 3> faceCamVerts = {
                    camVerts[cur.vertices[0].v],
                    camVerts[cur.vertices[1].v],
                    camVerts[cur.vertices[2].v]
            };

            std::vector<int> nearClipVerts;
            for (int i = 0; i < 3; ++i)
            {
                if (faceCamVerts[i].z >= -camera.get_near_clip()) nearClipVerts.push_back(i);
            }

            if (nearClipVerts.size() == 3) continue;

            if (nearClipVerts.empty())
            {
                draw_ss_triangle(cam_to_screen(faceCamVerts[0]), cam_to_screen(faceCamVerts[1]), cam_to_screen(faceCamVerts[2]), faceAttributes[0], faceAttributes[1], faceAttributes[2], camera, shader);
            }
            else if (nearClipVerts.size() == 1)
            {
                int iBehind = nearClipVerts[0];
                int i0 = (iBehind + 1) % 3;
                int i2 = (iBehind + 2) % 3;

                const auto& v_behind = faceCamVerts[iBehind];
                const auto& a_behind = faceAttributes[iBehind];
                const auto& v1 = faceCamVerts[i0];
                const auto& a1 = faceAttributes[i0];
                const auto& v2 = faceCamVerts[i2];
                const auto& a2 = faceAttributes[i2];

                float t1 = (-camera.get_near_clip() - v1.z) / (v_behind.z - v1.z);
                float t2 = (-camera.get_near_clip() - v2.z) / (v_behind.z - v2.z);

                Vector3 i1_cam = Vector3Lerp(v1, v_behind, t1);
                VertexAttributes i1_attr = lerp_attributes(a1, a_behind, t1);
                Vector3 i2_cam = Vector3Lerp(v2, v_behind, t2);
                VertexAttributes i2_attr = lerp_attributes(a2, a_behind, t2);

                Vector3 s_v1 = cam_to_screen(v1);
                Vector3 s_v2 = cam_to_screen(v2);
                Vector3 s1 = cam_to_screen(i1_cam);
                Vector3 s2 = cam_to_screen(i2_cam);

                draw_ss_triangle(s_v1, s_v2, s2, a1, a2, i2_attr, camera, shader);
                draw_ss_triangle(s_v1, s2, s1, a1, i2_attr, i1_attr, camera, shader);
            }
            else if (nearClipVerts.size() == 2)
            {
                int iInFront = 0;
                while (std::find(nearClipVerts.begin(), nearClipVerts.end(), iInFront) != nearClipVerts.end())
                {
                    iInFront++;
                }

                int iBehind0 = (iInFront + 1) % 3;
                int iBehind1 = (iInFront + 2) % 3;

                const auto& front_v = faceCamVerts[iInFront];
                const auto& front_a = faceAttributes[iInFront];
                const auto& back0_v = faceCamVerts[iBehind0];
                const auto& back0_a = faceAttributes[iBehind0];
                const auto& back1_v = faceCamVerts[iBehind1];
                const auto& back1_a = faceAttributes[iBehind1];

                float t0 = (-camera.get_near_clip() - front_v.z) / (back0_v.z - front_v.z);
                float t1 = (-camera.get_near_clip() - front_v.z) / (back1_v.z - front_v.z);

                Vector3 i0_cam = Vector3Lerp(front_v, back0_v, t0);
                VertexAttributes i0_attr = lerp_attributes(front_a, back0_a, t0);
                Vector3 i1_cam = Vector3Lerp(front_v, back1_v, t1);
                VertexAttributes i1_attr = lerp_attributes(front_a, back1_a, t1);

                draw_ss_triangle(cam_to_screen(front_v), cam_to_screen(i0_cam), cam_to_screen(i1_cam), front_a, i0_attr, i1_attr, camera, shader);
            }
        }
    }

    void PixelBuff::clear()
    {
        std::memset(depthbuffer.data(), 0xFF, depthbuffer.size() * sizeof(std::uint32_t));
        std::memset(framebuffer.data(), 0, framebuffer.size() * sizeof(std::uint32_t));
        //std::fill(depthbuffer.begin(), depthbuffer.end(), DEPTH_SCALE);
        //std::fill(depthbuffer.begin(), depthbuffer.end(), std::numeric_limits<float>::infinity());
        //std::fill(depthbuffer.begin(), depthbuffer.end(), std::numeric_limits<float>::infinity());
        //std::fill(framebuffer.begin(), framebuffer.end(), color);
    }

}