#pragma once
#include <vector>
#include <cstdint>
#include <raylib.h>
#include <raylib-cpp.hpp>
#include <string>
#include <thread>
#include <functional>
#include <atomic>
#include <algorithm>
#include "Model.h"

#define SIMDE_ENABLE_NATIVE_ALIASES
#include <simde/x86/sse.h>
#include <simde/x86/sse2.h>

#ifndef SIMDE_MM_SHUFFLE
#define SIMDE_MM_SHUFFLE(z, y, x, w) (((z) << 6) | ((y) << 4) | ((x) << 2) | (w))
#endif

namespace sky {

    namespace color
    {
        inline std::uint32_t pack(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a = 0xFF)
        {
            return (a << 24) | (b << 16) | (g << 8) | r;
        }

        inline std::array<float, 4> unpack(std::uint32_t color)
        {
            return std::array<float, 4>{
                    (float)((color >> 24) & 0xFF),
                    (float)((color >> 16) & 0xFF),
                    (float)((color >> 8) & 0xFF),
                    (float)(color & 0xFF)
            };
        }

        inline Vector4 unpack_vec4(std::uint32_t color)
        {
            return Vector4 {
                    (float)((color >> 24) & 0xFF),
                    (float)((color >> 16) & 0xFF),
                    (float)((color >> 8) & 0xFF),
                    (float)(color & 0xFF)
            };
        }
    }


    inline Vector3 Vector3NormalizeFast(const Vector3& v)
    {
        simde__m128 vec = simde_mm_set_ps(0.0f, v.z, v.y, v.x);
        simde__m128 dot = simde_mm_mul_ps(vec, vec);
        dot = simde_mm_add_ps(dot, simde_mm_shuffle_ps(dot, dot, 0x4E)); // z,y swap
        dot = simde_mm_add_ps(dot, simde_mm_shuffle_ps(dot, dot, 0xB1)); // x,?
        simde__m128 invLen = simde_mm_rsqrt_ps(dot);
        simde__m128 norm = simde_mm_mul_ps(vec, invLen);

        return {
                simde_mm_cvtss_f32(norm),
                simde_mm_cvtss_f32(simde_mm_shuffle_ps(norm, norm, 0x55)),
                simde_mm_cvtss_f32(simde_mm_shuffle_ps(norm, norm, 0xAA))
        };
    }

    static inline Vector3 barycentric_normalized_vec3(
            const Vector3& v0, const Vector3& v1, const Vector3& v2,
            float b0, float b1, float b2)
    {
        simde__m128 b = simde_mm_set_ps(0.0f, b2, b1, b0);

        auto pack = [](const Vector3& v) -> simde__m128
        {
            return simde_mm_set_ps(0.0f, v.z, v.y, v.x);
        };

        simde__m128 p0 = pack(v0);
        simde__m128 p1 = pack(v1);
        simde__m128 p2 = pack(v2);

        simde__m128 interp = simde_mm_add_ps(
                simde_mm_add_ps(
                        simde_mm_mul_ps(p0, simde_mm_shuffle_ps(b, b, 0x00)),
                        simde_mm_mul_ps(p1, simde_mm_shuffle_ps(b, b, 0x55))),
                simde_mm_mul_ps(p2, simde_mm_shuffle_ps(b, b, 0xAA))
        );

        simde__m128 dot = simde_mm_mul_ps(interp, interp);
        dot = simde_mm_add_ps(dot, simde_mm_shuffle_ps(dot, dot, 0x4E));
        dot = simde_mm_add_ps(dot, simde_mm_shuffle_ps(dot, dot, 0xB1));
        simde__m128 inv_len = simde_mm_rsqrt_ps(dot);
        simde__m128 norm = simde_mm_mul_ps(interp, inv_len);

        return {
                simde_mm_cvtss_f32(norm),
                simde_mm_cvtss_f32(simde_mm_shuffle_ps(norm, norm, 0x55)),
                simde_mm_cvtss_f32(simde_mm_shuffle_ps(norm, norm, 0xAA))
        };
    }

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

        Matrix get_projection_matrix(int screenWidth, int screenHeight) const;
    };

    using ShadeFunc = std::uint32_t(*) (const Vector3& normal,
                                        const Vector3& viewDir,
                                        const Vector3& lightDir);

    inline constexpr Vector3 kDefaultLightDir = { 0.5f, 0.5f, 0.5f };

    inline std::uint32_t shade_lambert(const Vector3& n,
                                       const Vector3& /*v*/,
                                       const Vector3& l = kDefaultLightDir)
    {
        float diff = std::max(0.0f, Vector3DotProduct(n, Vector3NormalizeFast(l)));
        std::uint8_t g = static_cast<std::uint8_t>(diff * 255.0f);
        return color::pack(g, g, g);
    }

    inline std::uint32_t shade_half_lambert(const Vector3& n,
                                            const Vector3& /*v*/,
                                            const Vector3& l = kDefaultLightDir)
    {
        float diff = Vector3DotProduct(n, Vector3NormalizeFast(l));
        diff = diff * 0.5f + 0.5f;
        diff = std::clamp(diff, 0.0f, 1.0f);
        std::uint8_t g = static_cast<std::uint8_t>(diff * 255.0f);
        return color::pack(g, g, g);
    }

    inline std::uint32_t shade_toon(const Vector3& n,
                                    const Vector3&,
                                    const Vector3& l = kDefaultLightDir)
    {
        float diff = std::max(0.0f, Vector3DotProduct(n, Vector3NormalizeFast(l)));
        if (diff > 0.80f) diff = 1.0f;
        else if (diff > 0.35f) diff = 0.6f;
        else if (diff > 0.10f) diff = 0.25f;
        else                   diff = 0.05f;
        std::uint8_t g = static_cast<std::uint8_t>(diff * 255.0f);
        return color::pack(g, g, g);
    }

    inline std::uint32_t shade_phong(const Vector3& n,
                                     const Vector3& v,
                                     const Vector3& l = kDefaultLightDir)
    {
        Vector3 ln = Vector3NormalizeFast(l);
        float diff = std::max(0.0f, Vector3DotProduct(n, ln));

        Vector3 r = Vector3Subtract(Vector3Scale(n, 2.f * diff), ln);
        float spec = std::pow(std::max(0.0f, Vector3DotProduct(r, v)), 16.0f);

        auto to8 = [](float x) { return static_cast<std::uint8_t>(std::clamp(x, 0.f, 1.f) * 255.f); };
        std::uint8_t c = to8(diff + spec);
        return color::pack(c, c, c);
    }

    inline std::uint32_t shade_rim(const Vector3& n,
                                   const Vector3& v,
                                   const Vector3& l = kDefaultLightDir)
    {
        float rim = std::pow(std::clamp(1.0f - std::fabs(Vector3DotProduct(n, v)), 0.0f, 1.0f), 3.0f);
        float diff = std::max(0.0f, Vector3DotProduct(n, Vector3NormalizeFast(l)));

        std::uint8_t r = static_cast<std::uint8_t>(std::clamp(diff + rim * 0.8f, 0.f, 1.f) * 255.f);
        std::uint8_t g = static_cast<std::uint8_t>(std::clamp(diff + rim * 0.4f, 0.f, 1.f) * 255.f);
        std::uint8_t b = static_cast<std::uint8_t>(std::clamp(diff + rim, 0.f, 1.f) * 255.f);
        return color::pack(r, g, b);
    }

    struct VertexAttributes
    {
        Vector3 world_normal;
        Vector3 world_pos;
    };

    class PixelBuff
    {
        int WIDTH, HEIGHT;
        //std::vector<float> depthbuffer;
        std::vector<std::uint32_t> depthbuffer; // now using fixed-point

        void set(int x, int y, std::uint32_t color);
        void draw_ss_triangle(const Vector3& p1, const Vector3& p2, const Vector3& p3, const VertexAttributes& a1, const VertexAttributes& a2, const VertexAttributes& a3, const Camera& camera, ShadeFunc shader);

    public:
        //void clear(std::uint32_t color);
        void clear();
        void draw_model(const Camera& camera, const Model& model, ShadeFunc shader = shade_lambert);
        PixelBuff(int WIDTH, int HEIGHT);

        std::vector<std::uint32_t> framebuffer;
        std::vector<float>  cocMap;
        std::vector<Vector3> colourLin;
    };
}