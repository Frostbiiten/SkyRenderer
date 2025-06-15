#include "PixelBuff.h"

// STD
#include <algorithm>
#include <iostream>
#include <execution>
#include <string_view>
#include <cstring>
#include <limits>
#include <vector>
#include <mutex>

// RAYLIB
#include <raylib-cpp.hpp>

// SKY
#include <sky/Model.h>
#include <sky/Util.h>

// simde
#include <simde/x86/sse.h>

#define PIXELBUFF_ENABLE_SIMD

namespace sky
{
    // Just a standard lerp for fields in VertexAttributes
    inline VertexAttributes lerp_attributes(const VertexAttributes& a,
                                            const VertexAttributes& b,
                                            float t)
    {
        return {
                Vector3Lerp(a.world_normal, b.world_normal, t),
                Vector3Lerp(a.world_pos, b.world_pos, t)
        };
    }

    // Constructor
    PixelBuff::PixelBuff(int WIDTH, int HEIGHT)
            : WIDTH(WIDTH), HEIGHT(HEIGHT),

            // Screen drawing buffers
            framebuffer(WIDTH * HEIGHT, 0),
            depthbuffer(WIDTH * HEIGHT, DEPTH_SCALE),

              // Preallocate vert buffer capacity. These can still resize later.
            modelTransformedVerts(8192),
            modelTransformedNormals(8192),
            camVerts(8192)
            {};

    void PixelBuff::set(int x, int y, std::uint32_t color)
    {
        framebuffer[y * WIDTH + x] = color;
    }

    void PixelBuff::set(int idx, std::uint32_t color)
    {
        framebuffer[idx] = color;
    }

    bool is_flat_shaded(const VertexAttributes& a1,
                        const VertexAttributes& a2,
                        const VertexAttributes& a3)
    {
        constexpr float epsilon = 1e-5f;
        return  Vector3NearEqual(a1.world_normal, a2.world_normal, epsilon)
                && Vector3NearEqual(a1.world_normal, a3.world_normal, epsilon)
                && Vector3NearEqual(a1.world_pos, a2.world_pos, epsilon)
                && Vector3NearEqual(a1.world_pos, a3.world_pos, epsilon);
    }

    // per-pixel shading: compute intensive
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
                        //framebuffer[idx] = color;
                        set(idx, color);
                    }
                }

                w0 += x_step0;
                w1 += x_step1;
                w2 += x_step2;
            }

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

        modelTransformedVerts.resize(verts.size());
        for (size_t i = 0; i < verts.size(); ++i)
        {
            modelTransformedVerts[i] = Vector3Transform(verts[i], modelMx);
        }

        modelTransformedNormals.resize(normals.size());
        for (size_t i = 0; i < normals.size(); ++i)
        {
            Vector4 n = { normals[i].x, normals[i].y, normals[i].z, 0.0f };
            Vector4 tn = Vector4Transform(n, normalMx);
            modelTransformedNormals[i] = Vector3Normalize({ tn.x, tn.y, tn.z });
        }

        camVerts.resize(verts.size());
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

            if (nearClipVerts.empty()) [[likely]]
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

    // flat drawing: no interpolation; much faster
    void PixelBuff::draw_ss_triangle_flat(const Vector3& p1, const Vector3& p2, const Vector3& p3, std::uint32_t  faceColour)
    {
        float area = (p2.x - p1.x) * (p3.y - p1.y) -
                     (p2.y - p1.y) * (p3.x - p1.x);
        if (area >= 0.0f) return;

        int minX = std::max((int)std::floor(std::min({ p1.x, p2.x, p3.x })), 0);
        int maxX = std::min((int)std::ceil(std::max({ p1.x, p2.x, p3.x })), WIDTH - 1);
        int minY = std::max((int)std::floor(std::min({ p1.y, p2.y, p3.y })), 0);
        int maxY = std::min((int)std::ceil(std::max({ p1.y, p2.y, p3.y })), HEIGHT - 1);

        float xStep0 = p2.y - p3.y, yStep0 = p3.x - p2.x;
        float xStep1 = p3.y - p1.y, yStep1 = p1.x - p3.x;
        float xStep2 = p1.y - p2.y, yStep2 = p2.x - p1.x;

        Vector3 pStart { (float)minX + 0.5f, (float)minY + 0.5f, 0.f };
        float w0Row = (p3.x - p2.x) * (pStart.y - p2.y) -
                      (p3.y - p2.y) * (pStart.x - p2.x);
        float w1Row = (p1.x - p3.x) * (pStart.y - p3.y) -
                      (p1.y - p3.y) * (pStart.x - p3.x);
        float w2Row = (p2.x - p1.x) * (pStart.y - p1.y) -
                      (p2.y - p1.y) * (pStart.x - p1.x);

        float invArea = 1.0f / area;

#ifdef PIXELBUFF_ENABLE_SIMD
        simde__m128 pz = simde_mm_set_ps(0.0f, p3.z, p2.z, p1.z);
#endif

        for (int y = minY; y <= maxY; ++y)
        {
            float w0 = w0Row, w1 = w1Row, w2 = w2Row;

            for (int x = minX; x <= maxX; ++x)
            {
                if (w0 <= 0 && w1 <= 0 && w2 <= 0)
                {
                    float b0 = w0 * invArea;
                    float b1 = w1 * invArea;
                    float b2 = w2 * invArea;

#ifdef PIXELBUFF_ENABLE_SIMD
                    simde__m128 bc = simde_mm_set_ps(0.0f, b2, b1, b0);
                    float zf = dot4(bc, pz);
#else
                    float zf = b0 * p1.z + b1 * p2.z + b2 * p3.z;
#endif
                    std::uint32_t z = float_to_fixed_depth(zf);
                    int idx = y * WIDTH + x;

                    if (z < depthbuffer[idx])
                    {
                        depthbuffer[idx] = z;
                        //framebuffer[idx] = faceColour;
                        set(idx, faceColour);
                    }
                }
                w0 += xStep0; w1 += xStep1; w2 += xStep2;
            }
            w0Row += yStep0; w1Row += yStep1; w2Row += yStep2;
        }
    }
    void PixelBuff::draw_model_flat(const Camera& camera, const Model& model, ShadeFunc shader)
    {
        const auto& faces = model.get_faces();
        const auto& verts = model.get_vertices();

        Matrix modelMx = model.get_transform();
        Matrix viewMx = camera.get_view_matrix();
        Matrix projMx = camera.get_projection_matrix(WIDTH, HEIGHT);

        modelTransformedVerts.resize(verts.size());
        camVerts.resize(verts.size());

        for (std::size_t i = 0; i < verts.size(); ++i)
        {
            modelTransformedVerts[i] = Vector3Transform(verts[i], modelMx);
            camVerts[i] = Vector3Transform(modelTransformedVerts[i], viewMx);
        }

        auto cam_to_screen = [&](const Vector3& c) -> Vector3
        {
            Vector4 clip = Vector4Transform({ c.x, c.y, c.z, 1.0f }, projMx);
            if (clip.w == 0.0f) return { 0, 0, std::numeric_limits<float>::infinity() };
            float iw = 1.0f / clip.w;
            return { (clip.x * iw * 0.5f + 0.5f) * WIDTH,
                     (1.0f - (clip.y * iw * 0.5f + 0.5f)) * HEIGHT,
                     clip.z * iw };
        };

        for (const auto& f : faces)
        {
            Vector3 w0 = modelTransformedVerts[f.vertices[0].v];
            Vector3 w1 = modelTransformedVerts[f.vertices[1].v];
            Vector3 w2 = modelTransformedVerts[f.vertices[2].v];

            Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(w1, w0),
                                                             Vector3Subtract(w2, w0)));
            Vector3 c = { (w0.x + w1.x + w2.x) / 3.0f,
                          (w0.y + w1.y + w2.y) / 3.0f,
                          (w0.z + w1.z + w2.z) / 3.0f };
            Vector3 viewDir = Vector3Normalize(Vector3Subtract(camera.position, c));
            std::uint32_t faceCol = shader(n, viewDir, kDefaultLightDir);

            Vector3 cv0 = camVerts[f.vertices[0].v];
            Vector3 cv1 = camVerts[f.vertices[1].v];
            Vector3 cv2 = camVerts[f.vertices[2].v];

            std::array<int, 3> behind {};
            int bc = 0;
            if (cv0.z >= -camera.get_near_clip()) behind[bc++] = 0;
            if (cv1.z >= -camera.get_near_clip()) behind[bc++] = 1;
            if (cv2.z >= -camera.get_near_clip()) behind[bc++] = 2;
            if (bc == 3) continue;

            if (bc == 0) [[likely]]
            {
                draw_ss_triangle_flat(cam_to_screen(cv0), cam_to_screen(cv1),
                                      cam_to_screen(cv2), faceCol);
            }
            else if (bc == 1)
            {
                int ib = behind[0];
                int i0 = (ib + 1) % 3;
                int i2 = (ib + 2) % 3;

                const Vector3& vb = camVerts[f.vertices[ib].v];
                const Vector3& v1 = camVerts[f.vertices[i0].v];
                const Vector3& v2 = camVerts[f.vertices[i2].v];

                float t1 = (-camera.get_near_clip() - v1.z) / (vb.z - v1.z);
                float t2 = (-camera.get_near_clip() - v2.z) / (vb.z - v2.z);

                Vector3 i1 = Vector3Lerp(v1, vb, t1);
                Vector3 i2p = Vector3Lerp(v2, vb, t2);

                draw_ss_triangle_flat(cam_to_screen(v1), cam_to_screen(v2),
                                      cam_to_screen(i2p), faceCol);
                draw_ss_triangle_flat(cam_to_screen(v1), cam_to_screen(i2p),
                                      cam_to_screen(i1), faceCol);
            }
            else
            {
                int ifr = (behind[0] == 0 || behind[1] == 0) ? ((behind[0] == 1 || behind[1] == 1) ? 2 : 1) : 0;
                int ib0 = (ifr + 1) % 3;
                int ib1 = (ifr + 2) % 3;

                const Vector3& vf = camVerts[f.vertices[ifr].v];
                const Vector3& vb0 = camVerts[f.vertices[ib0].v];
                const Vector3& vb1 = camVerts[f.vertices[ib1].v];

                float t0 = (-camera.get_near_clip() - vf.z) / (vb0.z - vf.z);
                float t1 = (-camera.get_near_clip() - vf.z) / (vb1.z - vf.z);

                Vector3 i0 = Vector3Lerp(vf, vb0, t0);
                Vector3 i1 = Vector3Lerp(vf, vb1, t1);

                draw_ss_triangle_flat(cam_to_screen(vf), cam_to_screen(i0),
                                      cam_to_screen(i1), faceCol);
            }
        }
    }

    // gouraud drawing: interpolates lighting on a per-vertex basis
    void PixelBuff::draw_ss_triangle_gouraud(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& c1, const Vector3& c2, const Vector3& c3)
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

#ifdef PIXELBUFF_ENABLE_SIMD
        simde__m128 pz = simde_mm_set_ps(0.0f, p3.z, p2.z, p1.z);
        simde__m128 c_r = simde_mm_set_ps(0.0f, c3.x, c2.x, c1.x);
        simde__m128 c_g = simde_mm_set_ps(0.0f, c3.y, c2.y, c1.y);
        simde__m128 c_b = simde_mm_set_ps(0.0f, c3.z, c2.z, c1.z);
#endif

        for (int y = min_y; y <= max_y; ++y)
        {
            float w0 = w0_row;
            float w1 = w1_row;
            float w2 = w2_row;

            for (int x = min_x; x <= max_x; ++x)
            {
                if (w0 <= 0 && w1 <= 0 && w2 <= 0)
                {
                    float b0 = w0 * inv_area;
                    float b1 = w1 * inv_area;
                    float b2 = w2 * inv_area;

#ifdef PIXELBUFF_ENABLE_SIMD
                    simde__m128 bc = simde_mm_set_ps(0.0f, b2, b1, b0);
                    float z_interp = dot4(bc, pz);
#else
                    float z_interp = b0 * p1.z + b1 * p2.z + b2 * p3.z;
#endif
                    int idx = y * WIDTH + x;
                    std::uint32_t z_fixed = float_to_fixed_depth(z_interp);

                    if (z_fixed < depthbuffer[idx])
                    {
                        depthbuffer[idx] = z_fixed;
#ifdef PIXELBUFF_ENABLE_SIMD
                        float r = dot4(bc, c_r);
                        float g = dot4(bc, c_g);
                        float b = dot4(bc, c_b);
                        //framebuffer[idx] = color::pack(r, g, b);
                        set(idx, color::pack(r, g, b));
#else
                        //framebuffer[idx] = color::pack(b0 * c1.x + b1 * c2.x + b2 * c3.x, b0 * c1.y + b1 * c2.y + b2 * c3.y, b0 * c1.z + b1 * c2.z + b2 * c3.z);
                        set(idx, color::pack(b0 * c1.x + b1 * c2.x + b2 * c3.x, b0 * c1.y + b1 * c2.y + b2 * c3.y, b0 * c1.z + b1 * c2.z + b2 * c3.z);
#endif
                    }
                }
                w0 += x_step0;
                w1 += x_step1;
                w2 += x_step2;
            }
            w0_row += y_step0;
            w1_row += y_step1;
            w2_row += y_step2;
        }
    }
    void PixelBuff::draw_model_gouraud(const Camera& camera, const Model& model, ShadeFunc shader)
    {
        const auto& faces = model.get_faces();
        const auto& verts = model.get_vertices();
        const auto& normals = model.get_normals();

        Matrix modelMx = model.get_transform();
        Matrix viewMx = camera.get_view_matrix();
        Matrix projMx = camera.get_projection_matrix(WIDTH, HEIGHT);
        Matrix normalMx = MatrixTranspose(MatrixInvert(modelMx));

        modelTransformedVerts.resize(verts.size());
        modelTransformedNormals.resize(normals.size());
        camVerts.resize(verts.size());

        for (size_t i = 0; i < normals.size(); ++i)
        {
            Vector4 n = { normals[i].x, normals[i].y, normals[i].z, 0.0f };
            Vector4 tn = Vector4Transform(n, normalMx);
            modelTransformedNormals[i] = Vector3Normalize({ tn.x, tn.y, tn.z });
        }

        for (size_t i = 0; i < verts.size(); ++i)
        {
            modelTransformedVerts[i] = Vector3Transform(verts[i], modelMx);
            camVerts[i] = Vector3Transform(modelTransformedVerts[i], viewMx);
        }

        auto cam_to_screen = [&](const Vector3& c) -> Vector3
        {
            Vector4 clip = Vector4Transform({ c.x, c.y, c.z, 1.0f }, projMx);
            if (clip.w == 0.0f) return { 0, 0, std::numeric_limits<float>::infinity() };
            float inv_w = 1.0f / clip.w;
            return {
                    (clip.x * inv_w * 0.5f + 0.5f) * WIDTH,
                    (1.0f - (clip.y * inv_w * 0.5f + 0.5f)) * HEIGHT,
                    clip.z * inv_w
            };
        };

        for (const auto& cur_face : faces)
        {
            std::array<Vector3, 3> face_vertex_colors;
            for (int i = 0; i < 3; ++i)
            {
                const Vector3& world_pos = modelTransformedVerts[cur_face.vertices[i].v];
                const Vector3& world_normal = modelTransformedNormals[cur_face.vertices[i].vn];
                Vector3 viewDir = Vector3Normalize(Vector3Subtract(camera.position, world_pos));
                uint32_t packed_color = shader(world_normal, viewDir, kDefaultLightDir);
                face_vertex_colors[i] = color::unpack_vec3(packed_color);
            }

            std::array<Vector3, 3> face_cam_verts = {
                    camVerts[cur_face.vertices[0].v],
                    camVerts[cur_face.vertices[1].v],
                    camVerts[cur_face.vertices[2].v]
            };

            std::vector<int> near_clip_verts_indices;
            for (int i = 0; i < 3; ++i)
            {
                if (face_cam_verts[i].z >= -camera.get_near_clip())
                {
                    near_clip_verts_indices.push_back(i);
                }
            }

            if (near_clip_verts_indices.size() == 3) continue;

            if (near_clip_verts_indices.empty()) [[likely]]
            {
                draw_ss_triangle_gouraud(
                        cam_to_screen(face_cam_verts[0]), cam_to_screen(face_cam_verts[1]),
                        cam_to_screen(face_cam_verts[2]),
                        face_vertex_colors[0], face_vertex_colors[1], face_vertex_colors[2]
                );
            }
            else if (near_clip_verts_indices.size() == 1)
            {
                int i_behind = near_clip_verts_indices[0];
                int i0 = (i_behind + 1) % 3;
                int i2 = (i_behind + 2) % 3;

                const auto& v_behind = face_cam_verts[i_behind];
                const auto& c_behind = face_vertex_colors[i_behind];
                const auto& v1 = face_cam_verts[i0];
                const auto& c1 = face_vertex_colors[i0];
                const auto& v2 = face_cam_verts[i2];
                const auto& c2 = face_vertex_colors[i2];

                float t1 = (-camera.get_near_clip() - v1.z) / (v_behind.z - v1.z);
                float t2 = (-camera.get_near_clip() - v2.z) / (v_behind.z - v2.z);

                Vector3 i1_cam_pos = Vector3Lerp(v1, v_behind, t1);
                Vector3 i1_color = Vector3Lerp(c1, c_behind, t1);
                Vector3 i2_cam_pos = Vector3Lerp(v2, v_behind, t2);
                Vector3 i2_color = Vector3Lerp(c2, c_behind, t2);

                draw_ss_triangle_gouraud(
                        cam_to_screen(v1), cam_to_screen(v2), cam_to_screen(i2_cam_pos),
                        c1, c2, i2_color
                );
                draw_ss_triangle_gouraud(
                        cam_to_screen(v1), cam_to_screen(i2_cam_pos), cam_to_screen(i1_cam_pos),
                        c1, i2_color, i1_color
                );
            }
            else if (near_clip_verts_indices.size() == 2)
            {
                int i_in_front = 0;
                while (std::find(near_clip_verts_indices.begin(), near_clip_verts_indices.end(), i_in_front) != near_clip_verts_indices.end())
                {
                    i_in_front++;
                }

                int i_behind0 = (i_in_front + 1) % 3;
                int i_behind1 = (i_in_front + 2) % 3;

                const auto& front_v = face_cam_verts[i_in_front];
                const auto& front_c = face_vertex_colors[i_in_front];
                const auto& back0_v = face_cam_verts[i_behind0];
                const auto& back0_c = face_vertex_colors[i_behind0];
                const auto& back1_v = face_cam_verts[i_behind1];
                const auto& back1_c = face_vertex_colors[i_behind1];

                float t0 = (-camera.get_near_clip() - front_v.z) / (back0_v.z - front_v.z);
                float t1 = (-camera.get_near_clip() - front_v.z) / (back1_v.z - front_v.z);

                Vector3 i0_cam_pos = Vector3Lerp(front_v, back0_v, t0);
                Vector3 i0_color = Vector3Lerp(front_c, back0_c, t0);
                Vector3 i1_cam_pos = Vector3Lerp(front_v, back1_v, t1);
                Vector3 i1_color = Vector3Lerp(front_c, back1_c, t1);

                draw_ss_triangle_gouraud(
                        cam_to_screen(front_v), cam_to_screen(i0_cam_pos), cam_to_screen(i1_cam_pos),
                        front_c, i0_color, i1_color
                );
            }
        }
    }

    void PixelBuff::blit_depthbuffer()
    {
        for (int y = 0; y < HEIGHT; ++y)
        {
            for (int x = 0; x < WIDTH; ++x)
            {
                int idx = y * WIDTH + x;
                std::uint32_t depth_fixed = depthbuffer[idx];
                std::uint8_t c;

                if (depth_fixed >= DEPTH_SCALE)
                {
                    c = 0;
                }
                else
                {
                    float normalized_depth = static_cast<float>(depth_fixed) / DEPTH_SCALE;
                    float grayscale_value = (1.0f - normalized_depth) * 10.f;
                    c = static_cast<std::uint8_t>(std::min(grayscale_value * 255.0f, 255.f));
                }

                std::uint32_t grayscale_color = color::pack(c, c, c);
                //framebuffer[idx] = grayscale_color;
                set(idx, grayscale_color);
            }
        }
    }


    // for fun

    /*
    #include <simde/x86/avx2.h>
    #include <numeric>
    #include <cmath>
    #include <cstdint>

    static inline simde__m128 gather4f(const float* table, simde__m128i idx32)
    {
        alignas(16) int32_t i[4];
        simde_mm_store_si128(reinterpret_cast<simde__m128i*>(i), idx32);
        return simde_mm_set_ps(table[i[3]], table[i[2]], table[i[1]], table[i[0]]);
    }

    #if defined(SIMDE_X86_AVX2_NATIVE) || defined(SIMDE_X86_WASM_SIMD128_NATIVE)
    #define GATHER_PS(tbl, idx) simde_mm_i32gather_ps((tbl), (idx), 4)
    #else
    #define GATHER_PS(tbl, idx) gather4f((tbl), (idx))
    #endif

    void PixelBuff::apply_depth_blur(float radius, float)
    {
        static constexpr std::array<Vector2, 12> taps = {{
                                                                 {-0.326f,-0.406f},{-0.840f,-0.074f},{-0.696f, 0.457f},{-0.203f, 0.621f},
                                                                 { 0.962f,-0.195f},{ 0.473f,-0.480f},{ 0.519f, 0.767f},{ 0.185f,-0.893f},
                                                                 { 0.507f, 0.064f},{ 0.896f, 0.412f},{-0.322f,-0.933f},{-0.792f,-0.598f}
                                                         }};

        constexpr float kGamma     = 2.2f;
        constexpr float kInvGamma  = 1.0f / kGamma;
        constexpr int   kKernel    = 13;      // centre + 12 Poisson taps
        constexpr int   kVecWidth  = 4;       // 4 pixels = 128-bit SIMD

        // LUTs for correction
        static float        sRGB2LinF[256];
        static std::uint8_t Lin2SRGB8[4097];  // 12-bit linear to 8-bit sRGB
        static bool lutBuilt = []{
            for (int i = 0; i < 256; ++i)
                sRGB2LinF[i] = std::pow(i / 255.0f, kGamma);

            for (int i = 0; i <= 4096; ++i)
                Lin2SRGB8[i] = static_cast<std::uint8_t>(
                        std::pow(i / 4096.0f, kInvGamma) * 255.0f + 0.5f);
            return true;
        }();

        // scale poisson offsets by radius
        struct Offset { int dx, dy; };
        Offset ofs[kKernel];
        ofs[0] = {0, 0};
        for (std::size_t i = 0; i < taps.size(); ++i) {
            ofs[i + 1].dx = static_cast<int>(std::round(taps[i].x * radius));
            ofs[i + 1].dy = static_cast<int>(std::round(taps[i].y * radius));
        }
        const float invSamples = 1.0f / kKernel;

        // destination
        std::vector<std::uint32_t> out(framebuffer.size());

        // row indices
        static std::vector<int> rows(HEIGHT);
        if (rows[2] == 0) std::iota(rows.begin(), rows.end(), 0);

        // worker
        auto blur_row = [&](int y)
        {
            // 4px at a time
            for (int x = 0; x <= WIDTH - kVecWidth; x += kVecWidth)
            {
                simde__m128 accR = simde_mm_setzero_ps();
                simde__m128 accG = simde_mm_setzero_ps();
                simde__m128 accB = simde_mm_setzero_ps();

                for (const Offset& o : ofs)
                {
                    int sy = fast_clamp(y + o.dy, 0, HEIGHT - 1);
                    int base = sy * WIDTH + (x + o.dx);

                    std::uint32_t pix[4];
                    for (int l = 0; l < kVecWidth; ++l)
                    {
                        int sx = fast_clamp((x + o.dx) + l, 0, WIDTH - 1);
                        pix[l] = framebuffer[sy * WIDTH + sx];
                    }
                    simde__m128i pack = simde_mm_loadu_si128(
                            reinterpret_cast<const simde__m128i*>(pix));

                    simde__m128i r8 = simde_mm_and_si128(simde_mm_srli_epi32(pack,  0), simde_mm_set1_epi32(0xFF));
                    simde__m128i g8 = simde_mm_and_si128(simde_mm_srli_epi32(pack,  8), simde_mm_set1_epi32(0xFF));
                    simde__m128i b8 = simde_mm_and_si128(simde_mm_srli_epi32(pack, 16), simde_mm_set1_epi32(0xFF));

                    accR = simde_mm_add_ps(accR, GATHER_PS(sRGB2LinF, r8));
                    accG = simde_mm_add_ps(accG, GATHER_PS(sRGB2LinF, g8));
                    accB = simde_mm_add_ps(accB, GATHER_PS(sRGB2LinF, b8));
                }

                // average
                simde__m128 norm = simde_mm_set1_ps(invSamples);
                accR = simde_mm_mul_ps(accR, norm);
                accG = simde_mm_mul_ps(accG, norm);
                accB = simde_mm_mul_ps(accB, norm);

                float rF[4], gF[4], bF[4];
                simde_mm_storeu_ps(rF, accR);
                simde_mm_storeu_ps(gF, accG);
                simde_mm_storeu_ps(bF, accB);

                for (int l = 0; l < kVecWidth; ++l)
                {
                    auto encode = [&](float lin) -> std::uint8_t {
                        int idx = static_cast<int>(lin * 4096.0f + 0.5f);
                        idx = std::clamp(idx, 0, 4096);
                        return Lin2SRGB8[idx];
                    };

                    std::uint8_t r8 = encode(rF[l]);
                    std::uint8_t g8 = encode(gF[l]);
                    std::uint8_t b8 = encode(bF[l]);

                    out[y * WIDTH + (x + l)] =
                            static_cast<std::uint32_t>(r8)
                            | (static_cast<std::uint32_t>(g8) <<  8)
                            | (static_cast<std::uint32_t>(b8) << 16)
                            | 0xFF000000u;   // alpha = 255
                }
            }

            for (int x = (WIDTH & ~3); x < WIDTH; ++x)
            {
                float rAcc = 0.0f, gAcc = 0.0f, bAcc = 0.0f;

                for (const Offset& o : ofs)
                {
                    int sx = std::clamp(x + o.dx, 0, WIDTH  - 1);
                    int sy = std::clamp(y + o.dy, 0, HEIGHT - 1);
                    std::uint32_t c = framebuffer[sy * WIDTH + sx];

                    rAcc += sRGB2LinF[(c      ) & 0xFF];       // R
                    gAcc += sRGB2LinF[(c >>  8) & 0xFF];       // G
                    bAcc += sRGB2LinF[(c >> 16) & 0xFF];       // B
                }
                rAcc *= invSamples;
                gAcc *= invSamples;
                bAcc *= invSamples;

                auto encode = [&](float lin) -> std::uint8_t {
                    int idx = static_cast<int>(lin * 4096.0f + 0.5f);
                    idx = std::clamp(idx, 0, 4096);
                    return Lin2SRGB8[idx];
                };

                std::uint8_t r8 = encode(rAcc);
                std::uint8_t g8 = encode(gAcc);
                std::uint8_t b8 = encode(bAcc);

                out[y * WIDTH + x] =
                        static_cast<std::uint32_t>(r8)
                        | (static_cast<std::uint32_t>(g8) <<  8)
                        | (static_cast<std::uint32_t>(b8) << 16)
                        | 0xFF000000u;
            }
        };

        std::for_each(std::execution::par_unseq, rows.begin(), rows.end(), blur_row);

        framebuffer.swap(out);
    }

    */

    // fast screen buffer clears
    void PixelBuff::clear()
    {
        std::memset(depthbuffer.data(), 0xFF, depthbuffer.size() * sizeof(std::uint32_t));
        //std::memset(framebuffer.data(), 0, framebuffer.size() * sizeof(std::uint32_t));
        constexpr auto backdrop = color::pack(kAmbientColor.r / 3, kAmbientColor.g / 3, kAmbientColor.b / 3);
        std::fill(framebuffer.begin(), framebuffer.end(), backdrop);
    }
}