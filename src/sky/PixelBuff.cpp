
#include "PixelBuff.h"
#include "cassert"
#include <algorithm>
#include <raylib-cpp.hpp>
#include <string_view>
#include "Model.h"
#include "fmt/core.h"
#include <limits>

namespace sky
{
    static inline Vector4 Vector4Transform(const Vector4& v, const Matrix& m)
    {
        return {
                v.x * m.m0 + v.y * m.m4 + v.z * m.m8 + v.w * m.m12,
                v.x * m.m1 + v.y * m.m5 + v.z * m.m9 + v.w * m.m13,
                v.x * m.m2 + v.y * m.m6 + v.z * m.m10 + v.w * m.m14,
                v.x * m.m3 + v.y * m.m7 + v.z * m.m11 + v.w * m.m15
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

    PixelBuff::PixelBuff(int WIDTH, int HEIGHT)
            : WIDTH(WIDTH), HEIGHT(HEIGHT), framebuffer(WIDTH* HEIGHT, 0), depthbuffer(WIDTH* HEIGHT, std::numeric_limits<float>::infinity())
    {
    };

    void PixelBuff::set(int x, int y, std::uint32_t color)
    {
        assert(x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT);
        framebuffer[y * WIDTH + x] = color;
    }

    std::uint32_t PixelBuff::get(int x, int y)
    {
        assert(x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT);
        return framebuffer[y * WIDTH + x];
    }

    void PixelBuff::draw_triangle2D(int x1, int y1, int x2, int y2, int x3, int y3, std::int32_t color)
    {
        int min_x = std::max(std::min({ x1, x2, x3, WIDTH - 1 }), 0);
        int min_y = std::max(std::min({ y1, y2, y3, HEIGHT - 1 }), 0);
        int max_x = std::min(std::max({ x1, x2, x3, 0 }), WIDTH - 1);
        int max_y = std::min(std::max({ y1, y2, y3, 0 }), HEIGHT - 1);

        int A12 = y1 - y2, B12 = x2 - x1, C12 = x1 * y2 - y1 * x2;
        int A23 = y2 - y3, B23 = x3 - x2, C23 = x2 * y3 - y2 * x3;
        int A31 = y3 - y1, B31 = x1 - x3, C31 = x3 * y1 - y3 * x1;

        for (int y = min_y; y <= max_y; ++y)
        {
            for (int x = min_x; x <= max_x; ++x)
            {
                int w12 = A12 * x + B12 * y + C12;
                int w23 = A23 * x + B23 * y + C23;
                int w31 = A31 * x + B31 * y + C31;

                if (w12 >= 0 && w23 >= 0 && w31 >= 0)
                    set(x, y, color);
            }
        }
    }

    void PixelBuff::draw_ss_triangle(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Camera& camera, std::int32_t color)
    {
        float area = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        if (area >= 0.0f) return;

        float x1 = p1.x, y1 = p1.y, z1 = p1.z;
        float x2 = p2.x, y2 = p2.y, z2 = p2.z;
        float x3 = p3.x, y3 = p3.y, z3 = p3.z;

        int min_x = std::max((int)std::floor(std::min({ x1, x2, x3 })), 0);
        int max_x = std::min((int)std::ceil(std::max({ x1, x2, x3 })), WIDTH - 1);
        int min_y = std::max((int)std::floor(std::min({ y1, y2, y3 })), 0);
        int max_y = std::min((int)std::ceil(std::max({ y1, y2, y3 })), HEIGHT - 1);

        float x_step0 = y2 - y3; float y_step0 = x3 - x2;
        float x_step1 = y3 - y1; float y_step1 = x1 - x3;
        float x_step2 = y1 - y2; float y_step2 = x2 - x1;

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

            for (int x = min_x; x <= max_x; ++x)
            {
                if (w0 <= 0 && w1 <= 0 && w2 <= 0)
                {
                    float b0 = w0 * inv_area;
                    float b1 = w1 * inv_area;
                    float b2 = w2 * inv_area;

                    float z = b0 * z1 + b1 * z2 + b2 * z3;
                    int idx = y * WIDTH + x;
                    if (z < depthbuffer[idx])
                    {
                        depthbuffer[idx] = z;
                        framebuffer[idx] = color;
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

    void PixelBuff::draw_model(const Camera& camera, const Model& model)
    {
        const auto& faces = model.get_faces();
        const auto& verts = model.get_vertices();

        Matrix view = camera.get_view_matrix();
        Matrix proj = camera.get_projection_matrix(WIDTH, HEIGHT);
        Matrix modelMx = model.get_transform();
        Matrix modelView = MatrixMultiply(modelMx, view);

        // transform all vertices once and store them
        std::vector<Vector3> camVerts(verts.size());
        for (size_t i = 0; i < verts.size(); ++i)
        {
            camVerts[i] = Vector3Transform(verts[i], modelView);
        }

        auto cam_to_screen = [&](const Vector3& cam) -> Vector3
        {
            Vector4 clip = Vector4Transform({ cam.x, cam.y, cam.z, 1.0f }, proj);
            if (clip.w == 0.0f) return { 0,0, std::numeric_limits<float>::infinity() };
            float ndcX = clip.x / clip.w;
            float ndcY = clip.y / clip.w;
            float ndcZ = clip.z / clip.w;
            return {
                    (ndcX * 0.5f + 0.5f) * WIDTH,
                    (1.0f - (ndcY * 0.5f + 0.5f)) * HEIGHT,
                    ndcZ
            };
        };

        std::vector<int> nearClipVerts;
        nearClipVerts.reserve(3);

        // iterate through faces using pre-transformed vertices
        for (const auto& cur : faces)
        {
            std::array<Vector3, 3> faceCamVerts = {
                    camVerts[cur.vertices[0].v],
                    camVerts[cur.vertices[1].v],
                    camVerts[cur.vertices[2].v]
            };

            nearClipVerts.clear();

            Vector3 e1 = Vector3Subtract(verts[cur.vertices[1].v], verts[cur.vertices[0].v]);
            Vector3 e2 = Vector3Subtract(verts[cur.vertices[2].v], verts[cur.vertices[0].v]);
            Vector3 normal = Vector3Normalize(Vector3CrossProduct(e1, e2));

            Vector4 baseColor { 255, 255, 255, 255 };
            float light = std::max(Vector3DotProduct(normal, Vector3Normalize(Vector3 { 0.5f, 0.5f, 0.5f })) * 0.5f + 0.5f, 0.f);
            baseColor.x *= light; baseColor.y *= light; baseColor.z *= light;
            auto color = color::pack((int)baseColor.x, (int)baseColor.y, (int)baseColor.z, (int)baseColor.w);

            for (int i = 0; i < 3; ++i)
            {
                if (faceCamVerts[i].z >= -camera.get_near_clip()) nearClipVerts.push_back(i);
            }

            switch (nearClipVerts.size())
            {
                case 3:
                    break;

                case 2:
                {
                    int iBehind0 = nearClipVerts[0];
                    int iBehind1 = nearClipVerts[1];
                    int iInFront = 3 - (iBehind0 + iBehind1);

                    const auto& front = faceCamVerts[iInFront];
                    const auto& back0 = faceCamVerts[iBehind0];
                    const auto& back1 = faceCamVerts[iBehind1];

                    float t0 = (-camera.get_near_clip() - front.z) / (back0.z - front.z);
                    float t1 = (-camera.get_near_clip() - front.z) / (back1.z - front.z);

                    Vector3 i0_cam = Vector3Lerp(front, back0, t0);
                    Vector3 i1_cam = Vector3Lerp(front, back1, t1);

                    draw_ss_triangle(cam_to_screen(front), cam_to_screen(i0_cam), cam_to_screen(i1_cam), camera, color);
                    break;
                }

                case 1:
                {
                    int iBehind = nearClipVerts[0];
                    int i0 = (iBehind + 1) % 3;
                    int i2 = (iBehind + 2) % 3;

                    const auto& v0 = faceCamVerts[iBehind];
                    const auto& v1 = faceCamVerts[i0];
                    const auto& v2 = faceCamVerts[i2];

                    float t1 = (-camera.get_near_clip() - v1.z) / (v0.z - v1.z);
                    float t2 = (-camera.get_near_clip() - v2.z) / (v0.z - v2.z);

                    Vector3 i1_cam = Vector3Lerp(v1, v0, t1);
                    Vector3 i2_cam = Vector3Lerp(v2, v0, t2);

                    Vector3 s_i0 = cam_to_screen(v1);
                    Vector3 s_i2 = cam_to_screen(v2);
                    Vector3 s1 = cam_to_screen(i1_cam);
                    Vector3 s2 = cam_to_screen(i2_cam);

                    draw_ss_triangle(s_i0, s_i2, s2, camera, color);
                    draw_ss_triangle(s_i0, s2, s1, camera, color);
                    break;
                }

                case 0:
                default:
                    draw_ss_triangle(
                            cam_to_screen(faceCamVerts[0]),
                            cam_to_screen(faceCamVerts[1]),
                            cam_to_screen(faceCamVerts[2]),
                            camera,
                            color
                    );
                    break;
            }
        }
    }

    void PixelBuff::clear(std::uint32_t color)
    {
        std::fill(depthbuffer.begin(), depthbuffer.end(), std::numeric_limits<float>::infinity());
        std::fill(framebuffer.begin(), framebuffer.end(), color);
    }
}