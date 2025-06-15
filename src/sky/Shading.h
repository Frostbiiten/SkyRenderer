#pragma once
#include <raylib.h>
#include <cstdint>
#include <algorithm>
#include <sky/Util.h>
#include "PixelBuff.h"

namespace sky
{
    using ShadeFunc = std::uint32_t(*) (const Vector3& normal,
                                        const Vector3& viewDir,
                                        const Vector3& lightDir);

    inline constexpr Vector3 kDefaultLightDir { 0.57735f, 0.57735f, -0.57735f };
    inline constexpr Color   kAmbientColor    { 5, 12, 22, 255 };

    inline std::uint32_t blend_with_ambient(float intensity)
    {
        auto clamp8 = [](int x) { return std::clamp(x, 0, 255); };

        std::uint8_t r = clamp8(kAmbientColor.r + int(intensity * 255.0f));
        std::uint8_t g = clamp8(kAmbientColor.g + int(intensity * 255.0f));
        std::uint8_t b = clamp8(kAmbientColor.b + int(intensity * 255.0f));

        return color::pack(r, g, b);
    }

    inline std::uint32_t shade_lambert(const Vector3& n,
                                       const Vector3& /*v*/,
                                       const Vector3& l = kDefaultLightDir)
    {
        float diff = std::max(0.0f, Vector3DotProduct(n, Vector3NormalizeFast(l)));
        return blend_with_ambient(diff);
    }

    inline std::uint32_t shade_half_lambert(const Vector3& n,
                                            const Vector3& /*v*/,
                                            const Vector3& l = kDefaultLightDir)
    {
        float diff = Vector3DotProduct(n, Vector3NormalizeFast(l));
        diff = diff * 0.5f + 0.5f;
        return blend_with_ambient(fast_clamp(diff, 0.0f, 1.0f));
    }

    inline std::uint32_t shade_toon(const Vector3& n,
                                    const Vector3&,
                                    const Vector3& l = kDefaultLightDir)
    {
        float diff = std::max(0.0f, Vector3DotProduct(n, Vector3NormalizeFast(l)));
        if      (diff > 0.80f) diff = 1.0f;
        else if (diff > 0.35f) diff = 0.6f;
        else if (diff > 0.10f) diff = 0.25f;
        else                   diff = 0.05f;
        return blend_with_ambient(diff);
    }

    inline std::uint32_t shade_phong(const Vector3& n,
                                     const Vector3& v,
                                     const Vector3& l = kDefaultLightDir)
    {
        Vector3 ln = Vector3NormalizeFast(l);
        float diff = std::max(0.0f, Vector3DotProduct(n, ln));

        Vector3 r = Vector3Subtract(Vector3Scale(n, 2.f * diff), ln);
        float spec = std::pow(std::max(0.0f, Vector3DotProduct(r, v)), 16.0f);

        return blend_with_ambient(fast_clamp(diff + spec, 0.0f, 1.0f));
    }

    inline std::uint32_t shade_rim(const Vector3& n,
                                   const Vector3& v,
                                   const Vector3& l = kDefaultLightDir)
    {
        float rim  = std::pow(fast_clamp(1.0f - std::fabs(Vector3DotProduct(n, v)), 0.0f, 1.0f), 3.0f);
        float diff = std::max(0.0f, Vector3DotProduct(n, Vector3NormalizeFast(l)));

        float r = fast_clamp((diff + rim * 0.8f), 0.f, 1.f);
        float g = fast_clamp((diff + rim * 0.4f), 0.f, 1.f);
        float b = fast_clamp((diff + rim * 1.0f), 0.f, 1.f);

        std::uint8_t rf = std::clamp(int(kAmbientColor.r + r * 255.0f), 0, 255);
        std::uint8_t gf = std::clamp(int(kAmbientColor.g + g * 255.0f), 0, 255);
        std::uint8_t bf = std::clamp(int(kAmbientColor.b + b * 255.0f), 0, 255);

        return color::pack(rf, gf, bf);
    }
}
