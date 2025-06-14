#pragma once
#include <vector>
#include <cstdint>
#include <raylib.h>
#include <raylib-cpp.hpp>
#include <string>
#include "Model.h"

namespace sky {

    namespace color
    {
        inline std::uint32_t pack(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a = 0xFF)
        {
            return (a << 24) | (r << 16) | (g << 8) | b;
        }

        inline std::array<float, 4> unpack(std::uint32_t color)
        {
            return std::array<float, 4>{
                    (float)((color >> 24) & 0xFF),
                    (float)((color >> 16) & 0xFF),
                    (float)((color >> 8)  & 0xFF),
                    (float)( color        & 0xFF)
            };
        }

        inline Vector4 unpack_vec4(std::uint32_t color)
        {
            return Vector4{
                    (float)((color >> 24) & 0xFF),
                    (float)((color >> 16) & 0xFF),
                    (float)((color >> 8)  & 0xFF),
                    (float)( color        & 0xFF)
            };
        }
    }

    struct Camera
    {
        Vector3 position;
        Quaternion rotation;
        //Matrix transform;
        float fov;
        float near_clip = 0.01f;
        float far_clip = 10000.f;

        public:
        explicit Camera(float fov = 60, Vector3 position = Vector3(0, 0, 00), Quaternion rotation = QuaternionIdentity());

        [[nodiscard]] Matrix get_transform();
        [[nodiscard]] Matrix get_view_matrix() const;
        [[nodiscard]] float get_near_clip() const;
        [[nodiscard]] float get_far_clip() const;

        Matrix get_projection_matrix(int screenWidth, int screenHeight) const;
    };

    class PixelBuff {
        int WIDTH, HEIGHT;
        // ARGB
        std::vector<float> depthbuffer;

        inline void set(int x, int y, std::uint32_t color);
        inline std::uint32_t get(int x, int y);

        // expects points to be given clockwise.
        void draw_triangle2D(int x1, int y1, int x2, int y2, int x3, int y3, std::int32_t color = 0xFFFFFFFF);
        void draw_triangle2D(const Vector2& p1, const Vector2& p2, const Vector2& p3, std::int32_t color = 0xFFFFFFFF);

        void draw_ss_triangle_fixed(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Camera&, std::int32_t color = 0xFFFFFFFF);
        void draw_ss_triangle(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Camera& camera, std::int32_t color = 0xFFFFFFFF);

    public:
        void clear(std::uint32_t color);
        void draw_model(const Camera& camera, const Model& model);

        PixelBuff(int WIDTH, int HEIGHT);

        std::vector<std::uint32_t> framebuffer;
    };
}