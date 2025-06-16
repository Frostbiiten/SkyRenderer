#pragma once

// std
#include <numbers>
#include <cstdint>
#include <cmath>
#include <array>

// Raylib
#include <raylib.h>

// SIMD-E
#define SIMDE_ENABLE_NATIVE_ALIASES
#include <simde/x86/sse.h>
#include <simde/x86/sse2.h>

#ifndef SIMDE_MM_SHUFFLE
#define SIMDE_MM_SHUFFLE(z, y, x, w) (((z) << 6) | ((y) << 4) | ((x) << 2) | (w))
#endif

namespace sky
{
    namespace color
    {
        constexpr inline std::uint32_t pack(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a = 0xFF)
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

        inline Vector3 unpack_vec3(std::uint32_t color)
        {
            return Vector3 {
                    (float)((color >> 16) & 0xFF),
                    (float)((color >> 8) & 0xFF),
                    (float)(color & 0xFF)
            };
        }

        inline float srgb_to_lin(float c)
        {
            return (c <= 0.04045f) ? c * (1.0f / 12.92f)
                                   : std::pow((c + 0.055f) * (1.0f / 1.055f), 2.4f);
        }

        inline float lin_to_srgb(float c)
        {
            c = std::clamp(c, 0.0f, 1.0f);
            return (c <= 0.0031308f) ? c * 12.92f
                                     : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
        }
    }

    // SIMD optimized Vector normalization
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

    inline float dot4(simde__m128 a, simde__m128 b)
    {
        simde__m128 mul = simde_mm_mul_ps(a, b);
        simde__m128 shuf1 = simde_mm_shuffle_ps(mul, mul, SIMDE_MM_SHUFFLE(2, 3, 0, 1));
        simde__m128 sum1 = simde_mm_add_ps(mul, shuf1);
        simde__m128 shuf2 = simde_mm_shuffle_ps(sum1, sum1, SIMDE_MM_SHUFFLE(1, 0, 3, 2));
        simde__m128 sum2 = simde_mm_add_ps(sum1, shuf2);
        return simde_mm_cvtss_f32(sum2);
    }

    inline Vector4 Vector4Transform(const Vector4& v, const Matrix& m)
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

    inline bool Vector3NearEqual(const Vector3& a, const Vector3& b, float eps)
    {
        return std::abs(a.x - b.x) < eps &&
               std::abs(a.y - b.y) < eps &&
               std::abs(a.z - b.z) < eps;
    }

    inline float fast_clamp(float v, float min, float max)
    {
        return std::min(max, std::max(min, v));
    }

    // [0, 1] mapped to [0, 2^24-1]
    inline constexpr std::uint32_t DEPTH_SCALE = (1 << 24) - 1;
    inline std::uint32_t float_to_fixed_depth(float z)
    {
        // z = fast_clamp(z, 0.0f, 1.0f); not really necessary...
        return static_cast<std::uint32_t>(z * DEPTH_SCALE);
    }

    inline float fixed_to_float_depth(std::uint32_t d)
    {
        return float(d) / float(DEPTH_SCALE);
    }
}

