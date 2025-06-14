#pragma once
#include <vector>
#include <raylib.h>
#include <string_view>
#include <array>

namespace sky
{
    struct Vertex
    {
        std::size_t v;  // vertex index
        std::size_t vt; // tex coord index
        std::size_t vn; // normal index
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
