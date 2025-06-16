#include "PixelBuff.h"

// STD
#include <algorithm>
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
    PixelBuff::PixelBuff(int width, int height)
            : WIDTH(width), HEIGHT(height),

            // Screen drawing buffers
            frame_buffer(WIDTH * HEIGHT, 0),
            depth_buffer(WIDTH * HEIGHT, DEPTH_SCALE),

              // Preallocate vert buffer capacity. These can still resize later.
            model_transformed_verts(8192),
            model_transformed_normals(8192),
            cam_verts(8192)

            {};

    void PixelBuff::set(int x, int y, std::uint32_t color)
    {
        frame_buffer[y * WIDTH + x] = color;
    }

    void PixelBuff::set(int idx, std::uint32_t color)
    {
        frame_buffer[idx] = color;
    }

    // May use later to speed up smooth models with some flat faces
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

    inline Vector3 PixelBuff::cam_to_screen(const Vector3& camera_space_pos, const Matrix& projection_matrix) const
    {
        Vector4 clip_space_pos = Vector4Transform({ camera_space_pos.x, camera_space_pos.y, camera_space_pos.z, 1.0f }, projection_matrix);
        if (clip_space_pos.w == 0.0f) return { 0.0f, 0.0f, std::numeric_limits<float>::infinity() };

        float inverse_w = 1.0f / clip_space_pos.w;

        // Perspective divide to get NDC
        float ndc_x = clip_space_pos.x * inverse_w;
        float ndc_y = clip_space_pos.y * inverse_w;
        float ndc_z = clip_space_pos.z * inverse_w;

        // convert to screen coordinates
        return {
                (ndc_x * 0.5f + 0.5f) * WIDTH,
                (1.0f - (ndc_y * 0.5f + 0.5f)) * HEIGHT,
                ndc_z
        };
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

#ifdef PIXELBUFF_ENABLE_SIMD
        simde__m128 pz   = simde_mm_set_ps(0.0f, p3.z, p2.z, p1.z);
        simde__m128 n0   = pack_vec3(a1.world_normal);
        simde__m128 n1   = pack_vec3(a2.world_normal);
        simde__m128 n2   = pack_vec3(a3.world_normal);
        simde__m128 pos0 = pack_vec3(a1.world_pos);
        simde__m128 pos1 = pack_vec3(a2.world_pos);
        simde__m128 pos2 = pack_vec3(a3.world_pos);
#endif


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

                    if (z < depth_buffer[idx])
                    {
                        depth_buffer[idx] = z;

#ifdef PIXELBUFF_ENABLE_SIMD
                        simde__m128 bc = simde_mm_set_ps(0.0f, b2, b1, b0);

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

                        Vector3 normal = Vector3NormalizeFast(unpack_vec3(interp_n));
                        Vector3 world_pos = unpack_vec3(interp_p);
#else
                        Vector3 world_pos = Vector3Add(Vector3Add(
                            Vector3Scale(a1.world_pos, b0),
                            Vector3Scale(a2.world_pos, b1)),
                                                       Vector3Scale(a3.world_pos, b2));

                        Vector3 normal = Vector3NormalizeFast(Vector3Add(Vector3Add(
                            Vector3Scale(a1.world_normal, b0),
                            Vector3Scale(a2.world_normal, b1)),
                            Vector3Scale(a3.world_normal, b2)));
#endif
                        Vector3 view_dir = Vector3NormalizeFast(Vector3Subtract(camera.position, world_pos));
                        uint32_t color = shader(normal, view_dir, LIGHT_DIR);
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
        Matrix model_matrix = model.get_transform();
        Matrix normal_matrix = MatrixTranspose(MatrixInvert(model_matrix));

        model_transformed_verts.resize(verts.size());
        for (size_t i = 0; i < verts.size(); ++i)
        {
            model_transformed_verts[i] = Vector3Transform(verts[i], model_matrix);
        }

        model_transformed_normals.resize(normals.size());
        for (size_t i = 0; i < normals.size(); ++i)
        {
            Vector4 n = { normals[i].x, normals[i].y, normals[i].z, 0.0f };
            Vector4 tn = Vector4Transform(n, normal_matrix);
            model_transformed_normals[i] = Vector3NormalizeFast({tn.x, tn.y, tn.z });
        }

        cam_verts.resize(verts.size());
        for (size_t i = 0; i < verts.size(); ++i)
        {
            cam_verts[i] = Vector3Transform(model_transformed_verts[i], view);
        }

        for (const auto& cur : faces)
        {
            std::array<VertexAttributes, 3> face_attributes = {
                    VertexAttributes{model_transformed_normals[cur.vertices[0].vn], model_transformed_verts[cur.vertices[0].v]},
                    VertexAttributes{model_transformed_normals[cur.vertices[1].vn], model_transformed_verts[cur.vertices[1].v]},
                    VertexAttributes{model_transformed_normals[cur.vertices[2].vn], model_transformed_verts[cur.vertices[2].v]}
            };

            std::array<Vector3, 3> face_cam_verts = {
                    cam_verts[cur.vertices[0].v],
                    cam_verts[cur.vertices[1].v],
                    cam_verts[cur.vertices[2].v]
            };

            std::vector<int> near_clip_verts;
            for (int i = 0; i < 3; ++i)
            {
                if (face_cam_verts[i].z >= -camera.get_near_clip()) near_clip_verts.push_back(i);
            }

            if (near_clip_verts.size() == 3) continue;

            if (near_clip_verts.empty()) [[likely]]
            {
                draw_ss_triangle(cam_to_screen(face_cam_verts[0], proj), cam_to_screen(face_cam_verts[1], proj), cam_to_screen(face_cam_verts[2], proj), face_attributes[0], face_attributes[1], face_attributes[2], camera, shader);
            }
            else if (near_clip_verts.size() == 1)
            {
                int iBehind = near_clip_verts[0];
                int i0 = (iBehind + 1) % 3;
                int i2 = (iBehind + 2) % 3;

                const auto& v_behind = face_cam_verts[iBehind];
                const auto& a_behind = face_attributes[iBehind];
                const auto& v1 = face_cam_verts[i0];
                const auto& a1 = face_attributes[i0];
                const auto& v2 = face_cam_verts[i2];
                const auto& a2 = face_attributes[i2];

                float t1 = (-camera.get_near_clip() - v1.z) / (v_behind.z - v1.z);
                float t2 = (-camera.get_near_clip() - v2.z) / (v_behind.z - v2.z);

                Vector3 i1_cam = Vector3Lerp(v1, v_behind, t1);
                VertexAttributes i1_attr = lerp_attributes(a1, a_behind, t1);
                Vector3 i2_cam = Vector3Lerp(v2, v_behind, t2);
                VertexAttributes i2_attr = lerp_attributes(a2, a_behind, t2);

                Vector3 s_v1 = cam_to_screen(v1, proj);
                Vector3 s_v2 = cam_to_screen(v2, proj);
                Vector3 s1 = cam_to_screen(i1_cam, proj);
                Vector3 s2 = cam_to_screen(i2_cam, proj);

                draw_ss_triangle(s_v1, s_v2, s2, a1, a2, i2_attr, camera, shader);
                draw_ss_triangle(s_v1, s2, s1, a1, i2_attr, i1_attr, camera, shader);
            }
            else if (near_clip_verts.size() == 2)
            {
                int i_in_front = 0;
                while (std::find(near_clip_verts.begin(), near_clip_verts.end(), i_in_front) != near_clip_verts.end())
                {
                    i_in_front++;
                }

                int i_behind0 = (i_in_front + 1) % 3;
                int i_behind1 = (i_in_front + 2) % 3;

                const auto& front_v = face_cam_verts[i_in_front];
                const auto& front_a = face_attributes[i_in_front];
                const auto& back0_v = face_cam_verts[i_behind0];
                const auto& back0_a = face_attributes[i_behind0];
                const auto& back1_v = face_cam_verts[i_behind1];
                const auto& back1_a = face_attributes[i_behind1];

                float t0 = (-camera.get_near_clip() - front_v.z) / (back0_v.z - front_v.z);
                float t1 = (-camera.get_near_clip() - front_v.z) / (back1_v.z - front_v.z);

                Vector3 i0_cam = Vector3Lerp(front_v, back0_v, t0);
                VertexAttributes i0_attr = lerp_attributes(front_a, back0_a, t0);
                Vector3 i1_cam = Vector3Lerp(front_v, back1_v, t1);
                VertexAttributes i1_attr = lerp_attributes(front_a, back1_a, t1);

                draw_ss_triangle(cam_to_screen(front_v, proj), cam_to_screen(i0_cam, proj), cam_to_screen(i1_cam, proj), front_a, i0_attr, i1_attr, camera, shader);
            }
        }
    }

    // flat drawing: no interpolation; much faster
    void PixelBuff::draw_ss_triangle_flat(const Vector3& p1, const Vector3& p2, const Vector3& p3, std::uint32_t  faceColour)
    {
        float area = (p2.x - p1.x) * (p3.y - p1.y) -
                     (p2.y - p1.y) * (p3.x - p1.x);
        if (area >= 0.0f) return;

        int min_x = std::max((int)std::floor(std::min({p1.x, p2.x, p3.x })), 0);
        int max_x = std::min((int)std::ceil(std::max({p1.x, p2.x, p3.x })), WIDTH - 1);
        int min_y = std::max((int)std::floor(std::min({p1.y, p2.y, p3.y })), 0);
        int max_y = std::min((int)std::ceil(std::max({p1.y, p2.y, p3.y })), HEIGHT - 1);

        float x_step0 = p2.y - p3.y, yStep0 = p3.x - p2.x;
        float x_step1 = p3.y - p1.y, yStep1 = p1.x - p3.x;
        float x_step2 = p1.y - p2.y, yStep2 = p2.x - p1.x;

        Vector3 p_start {(float)min_x + 0.5f, (float)min_y + 0.5f, 0.f };
        float w0_row = (p3.x - p2.x) * (p_start.y - p2.y) -
                      (p3.y - p2.y) * (p_start.x - p2.x);
        float w1_row = (p1.x - p3.x) * (p_start.y - p3.y) -
                      (p1.y - p3.y) * (p_start.x - p3.x);
        float w2_row = (p2.x - p1.x) * (p_start.y - p1.y) -
                      (p2.y - p1.y) * (p_start.x - p1.x);

        float inv_area = 1.0f / area;

#ifdef PIXELBUFF_ENABLE_SIMD
        simde__m128 pz = simde_mm_set_ps(0.0f, p3.z, p2.z, p1.z);
#endif

        for (int y = min_y; y <= max_y; ++y)
        {
            float w0 = w0_row, w1 = w1_row, w2 = w2_row;

            for (int x = min_x; x <= max_x; ++x)
            {
                if (w0 <= 0 && w1 <= 0 && w2 <= 0)
                {
                    float b0 = w0 * inv_area;
                    float b1 = w1 * inv_area;
                    float b2 = w2 * inv_area;

#ifdef PIXELBUFF_ENABLE_SIMD
                    simde__m128 bc = simde_mm_set_ps(0.0f, b2, b1, b0);
                    float zf = dot4(bc, pz);
#else
                    float zf = b0 * p1.z + b1 * p2.z + b2 * p3.z;
#endif
                    std::uint32_t z = float_to_fixed_depth(zf);
                    int idx = y * WIDTH + x;

                    if (z < depth_buffer[idx])
                    {
                        depth_buffer[idx] = z;
                        set(idx, faceColour);
                    }
                }
                w0 += x_step0; w1 += x_step1; w2 += x_step2;
            }
            w0_row += yStep0; w1_row += yStep1; w2_row += yStep2;
        }
    }
    void PixelBuff::draw_model_flat(const Camera& camera, const Model& model, ShadeFunc shader)
    {
        const auto& faces = model.get_faces();
        const auto& verts = model.get_vertices();

        Matrix model_matrix = model.get_transform();
        Matrix view_matrix = camera.get_view_matrix();
        Matrix proj = camera.get_projection_matrix(WIDTH, HEIGHT);

        model_transformed_verts.resize(verts.size());
        cam_verts.resize(verts.size());

        for (std::size_t i = 0; i < verts.size(); ++i)
        {
            model_transformed_verts[i] = Vector3Transform(verts[i], model_matrix);
            cam_verts[i] = Vector3Transform(model_transformed_verts[i], view_matrix);
        }

        for (const auto& f : faces)
        {
            Vector3 w0 = model_transformed_verts[f.vertices[0].v];
            Vector3 w1 = model_transformed_verts[f.vertices[1].v];
            Vector3 w2 = model_transformed_verts[f.vertices[2].v];

            Vector3 n = Vector3NormalizeFast(Vector3CrossProduct(Vector3Subtract(w1, w0),
                                                             Vector3Subtract(w2, w0)));
            Vector3 c = { (w0.x + w1.x + w2.x) / 3.0f,
                          (w0.y + w1.y + w2.y) / 3.0f,
                          (w0.z + w1.z + w2.z) / 3.0f };
            Vector3 view_dir = Vector3NormalizeFast(Vector3Subtract(camera.position, c));
            std::uint32_t face_col = shader(n, view_dir, LIGHT_DIR);

            Vector3 cv0 = cam_verts[f.vertices[0].v];
            Vector3 cv1 = cam_verts[f.vertices[1].v];
            Vector3 cv2 = cam_verts[f.vertices[2].v];

            std::array<int, 3> behind {};
            int bc = 0;
            if (cv0.z >= -camera.get_near_clip()) behind[bc++] = 0;
            if (cv1.z >= -camera.get_near_clip()) behind[bc++] = 1;
            if (cv2.z >= -camera.get_near_clip()) behind[bc++] = 2;
            if (bc == 3) continue;

            if (bc == 0) [[likely]]
            {
                draw_ss_triangle_flat(cam_to_screen(cv0, proj), cam_to_screen(cv1, proj),
                                      cam_to_screen(cv2, proj), face_col);
            }
            else if (bc == 1)
            {
                int ib = behind[0];
                int i0 = (ib + 1) % 3;
                int i2 = (ib + 2) % 3;

                const Vector3& vb = cam_verts[f.vertices[ib].v];
                const Vector3& v1 = cam_verts[f.vertices[i0].v];
                const Vector3& v2 = cam_verts[f.vertices[i2].v];

                float t1 = (-camera.get_near_clip() - v1.z) / (vb.z - v1.z);
                float t2 = (-camera.get_near_clip() - v2.z) / (vb.z - v2.z);

                Vector3 i1 = Vector3Lerp(v1, vb, t1);
                Vector3 i2p = Vector3Lerp(v2, vb, t2);

                draw_ss_triangle_flat(cam_to_screen(v1, proj), cam_to_screen(v2, proj),
                                      cam_to_screen(i2p, proj), face_col);
                draw_ss_triangle_flat(cam_to_screen(v1, proj), cam_to_screen(i2p, proj),
                                      cam_to_screen(i1, proj), face_col);
            }
            else
            {
                int ifr = (behind[0] == 0 || behind[1] == 0) ? ((behind[0] == 1 || behind[1] == 1) ? 2 : 1) : 0;
                int ib0 = (ifr + 1) % 3;
                int ib1 = (ifr + 2) % 3;

                const Vector3& vf = cam_verts[f.vertices[ifr].v];
                const Vector3& vb0 = cam_verts[f.vertices[ib0].v];
                const Vector3& vb1 = cam_verts[f.vertices[ib1].v];

                float t0 = (-camera.get_near_clip() - vf.z) / (vb0.z - vf.z);
                float t1 = (-camera.get_near_clip() - vf.z) / (vb1.z - vf.z);

                Vector3 i0 = Vector3Lerp(vf, vb0, t0);
                Vector3 i1 = Vector3Lerp(vf, vb1, t1);

                draw_ss_triangle_flat(cam_to_screen(vf, proj), cam_to_screen(i0, proj),
                                      cam_to_screen(i1, proj), face_col);
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

                    if (z_fixed < depth_buffer[idx])
                    {
                        depth_buffer[idx] = z_fixed;
#ifdef PIXELBUFF_ENABLE_SIMD
                        float r = dot4(bc, c_r);
                        float g = dot4(bc, c_g);
                        float b = dot4(bc, c_b);
                        set(idx, color::pack(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)));
#else
                    set(idx, color::pack(
                        static_cast<uint8_t>(b0 * c1.x + b1 * c2.x + b2 * c3.x),
                        static_cast<uint8_t>(b0 * c1.y + b1 * c2.y + b2 * c3.y),
                        static_cast<uint8_t>(b0 * c1.z + b1 * c2.z + b2 * c3.z))
                    );
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

        Matrix model_matrix = model.get_transform();
        Matrix view_matrix = camera.get_view_matrix();
        Matrix proj = camera.get_projection_matrix(WIDTH, HEIGHT);
        Matrix normal_matrix = MatrixTranspose(MatrixInvert(model_matrix));

        model_transformed_verts.resize(verts.size());
        model_transformed_normals.resize(normals.size());
        cam_verts.resize(verts.size());

        for (size_t i = 0; i < normals.size(); ++i)
        {
            Vector4 n = { normals[i].x, normals[i].y, normals[i].z, 0.0f };
            Vector4 tn = Vector4Transform(n, normal_matrix);
            model_transformed_normals[i] = Vector3NormalizeFast({tn.x, tn.y, tn.z });
        }

        for (size_t i = 0; i < verts.size(); ++i)
        {
            model_transformed_verts[i] = Vector3Transform(verts[i], model_matrix);
            cam_verts[i] = Vector3Transform(model_transformed_verts[i], view_matrix);
        }

        auto cam_to_screen = [&](const Vector3& c) -> Vector3
        {
            Vector4 clip = Vector4Transform({ c.x, c.y, c.z, 1.0f }, proj);
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
                const Vector3& world_pos = model_transformed_verts[cur_face.vertices[i].v];
                const Vector3& world_normal = model_transformed_normals[cur_face.vertices[i].vn];
                Vector3 viewDir = Vector3NormalizeFast(Vector3Subtract(camera.position, world_pos));
                uint32_t packed_color = shader(world_normal, viewDir, LIGHT_DIR);

                face_vertex_colors[i] = {
                        (float)(packed_color & 0xFF),
                        (float)((packed_color >> 8) & 0xFF),
                        (float)((packed_color >> 16) & 0xFF)
                };
            }

            std::array<Vector3, 3> face_cam_verts = {
                    cam_verts[cur_face.vertices[0].v],
                    cam_verts[cur_face.vertices[1].v],
                    cam_verts[cur_face.vertices[2].v]
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
                std::uint32_t depth_fixed = depth_buffer[idx];
                std::uint8_t c;

                if (depth_fixed >= DEPTH_SCALE)
                {
                    c = 0;
                }
                else
                {
                    float normalized_depth = static_cast<float>(depth_fixed) / DEPTH_SCALE;
                    float grayscale_value = (1.0f - normalized_depth) * 10.f; // this is manual... just wanted to increase visibility for demos
                    c = static_cast<std::uint8_t>(std::min(grayscale_value * 255.0f, 255.f));
                }

                std::uint32_t grayscale_color = color::pack(c, c, c);
                set(idx, grayscale_color);
            }
        }
    }

    // clears screen buffers for redrawing
    void PixelBuff::clear()
    {
        std::memset(depth_buffer.data(), 0xFF, depth_buffer.size() * sizeof(std::uint32_t));
        constexpr auto backdrop = color::pack(AMBIENT_COL.r / 3, AMBIENT_COL.g / 3, AMBIENT_COL.b / 3); // make backdrop a bit darker
        std::fill(frame_buffer.begin(), frame_buffer.end(), backdrop);
    }

    std::vector<std::uint32_t> &PixelBuff::get_frame() {
        return frame_buffer;
    }
}