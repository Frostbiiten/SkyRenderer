#pragma once

// STD
#include <vector>
#include <string_view>
#include <array>

// RAYLIB
#include <raylib.h>

namespace sky
{
    struct Vertex
    {   // indexes of ...
        std::size_t v;  // vertex
        std::size_t vt; // tex coord
        std::size_t vn; // normal
    };

    struct Face
    {
        std::array<Vertex, 3> vertices;
    };

    class Model
    {
        std::vector<Vector3> vertices;
        std::vector<Vector3> normals;
        std::vector<Vector3> uvs;
        bool smoothing;

        std::vector<Face> faces;
        Matrix transform;
        void load(std::string_view name);

    public:
        Model(std::string_view name, Matrix transform);
        [[nodiscard]] const std::vector<Vector3>& get_vertices() const;
        [[nodiscard]] const std::vector<Vector3>& get_normals() const;
        [[nodiscard]] const std::vector<Vector3>& get_uvs() const ;
        [[nodiscard]] const std::vector<Face> &get_faces() const;
        [[nodiscard]] const Matrix &get_transform() const;

        Model &set_transform(Matrix transform);
    };
}
