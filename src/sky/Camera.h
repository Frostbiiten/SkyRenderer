#pragma once

// RAYLIB
#include <raylib.h>
#include <raylib-cpp.hpp>

namespace sky
{
    struct Camera
    {
        Vector3 position;
        Quaternion rotation;
        float fov;
        float near_clip = 0.1f;
        float far_clip = 1000.f;

    public:
        explicit Camera(float fov = 60, Vector3 position = Vector3(0, 0, 0), Quaternion rotation = QuaternionIdentity());

        [[nodiscard]] Matrix get_view_matrix() const;
        [[nodiscard]] float get_near_clip() const;
        [[nodiscard]] float get_far_clip() const;
        [[nodiscard]] Matrix get_projection_matrix(int screenWidth, int screenHeight) const;
    };
}
