#pragma once

// STD
#include <vector>
#include <cstdint>
#include <string>
#include <thread>
#include <functional>
#include <atomic>
#include <algorithm>

// RAYLIB
#include <raylib.h>
#include <raylib-cpp.hpp>

// SKY
#include <sky/Camera.h>
#include <sky/Shading.h>
#include <sky/Model.h>

namespace sky {

    struct VertexAttributes
    {
        Vector3 world_normal;
        Vector3 world_pos;
    };

    struct GouraudVert
    {
        Vector3 colOverW;  // RGB * invW
        float invW;        // 1 / clip.w
    };

    class PixelBuff
    {
        int WIDTH, HEIGHT;
        std::vector<std::uint32_t> depthbuffer; // now using fixed-point

        inline void set(int x, int y, std::uint32_t color);
        inline void set(int idx, std::uint32_t color);

        void draw_ss_triangle(const Vector3& p1,
                              const Vector3& p2,
                              const Vector3& p3,
                              const VertexAttributes& a1,
                              const VertexAttributes& a2,
                              const VertexAttributes& a3,
                              const Camera& camera,
                              ShadeFunc shader = shade_half_lambert);

        void draw_ss_triangle_flat(const Vector3& p1,
                                   const Vector3& p2,
                                   const Vector3& p3,
                                   std::uint32_t faceColour);

        void draw_ss_triangle_gouraud(const Vector3 &p1,
                                      const Vector3 &p2,
                                      const Vector3 &p3,
                                      const Vector3 &c1,
                                      const Vector3 &c2,
                                      const Vector3 &c3);

        std::vector<Vector3> modelTransformedVerts;
        std::vector<Vector3> modelTransformedNormals;
        std::vector<std::uint32_t> framebuffer;
        std::vector<Vector3> camVerts;

    public:
        PixelBuff(int WIDTH, int HEIGHT);

        void draw_model(const Camera& camera, const Model& model, ShadeFunc shader = shade_half_lambert);
        void draw_model_flat(const Camera& camera, const Model& model, ShadeFunc shader = shade_half_lambert);
        void draw_model_gouraud(const Camera &camera, const Model &model, ShadeFunc shader = shade_half_lambert);

        std::vector<std::uint32_t>& get_frame();
        void blit_depthbuffer();
        void clear();
    };
}