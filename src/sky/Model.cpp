// STD
#include <fstream>
#include <cassert>
#include <charconv>

// SKY
#include <sky/Model.h>

bool parse_floats(std::string_view str, std::initializer_list<float*> outputs)
{
    auto skip_ws = [](std::string_view& sv)
    {
        while (!sv.empty() && std::isspace(static_cast<unsigned char>(sv.front())))
            sv.remove_prefix(1);
    };

    for (float* out : outputs)
    {
        skip_ws(str);

        const char* begin = str.data();
        char* end = nullptr;

        errno = 0;
        float val = std::strtof(begin, &end);

        if (end == begin || errno == ERANGE)
            return false;

        *out = val;

        std::size_t consumed = static_cast<std::size_t>(end - begin);
        if (consumed > str.size()) return false;
        str.remove_prefix(consumed);
    }

    return true;
}


namespace sky
{
    Model::Model(std::string_view name, Matrix transform) : transform(transform)
    {
        load(name);

        // this isn't actually used right now...
        smoothing = false;
    }

    void Model::load(std::string_view name)
    {
        std::ifstream in{ name.data() };
        if (!in) throw std::runtime_error("Could not open OBJ file: " + std::string{name});

        std::string line;
        while (std::getline(in, line))
        {
            if (line.starts_with("v "))
            {
                vertices.resize(vertices.size() + 1);
                std::string_view sv = std::string_view(line).substr(2);
                parse_floats(sv, { &vertices.back().x, &vertices.back().y, &vertices.back().z });
            }
            else if (line.starts_with("vt "))
            {
                uvs.resize(uvs.size() + 1);
                std::string_view sv = std::string_view(line).substr(3);
                parse_floats(sv, { &uvs.back().x, &uvs.back().y });
            }
            else if (line.starts_with("vn "))
            {
                normals.resize(normals.size() + 1);
                std::string_view sv = std::string_view(line).substr(3);
                parse_floats(sv, { &normals.back().x, &normals.back().y, &normals.back().z });
            }
            else if (line.starts_with("f "))
            {
                std::string_view face_str = std::string_view(line).substr(2);
                std::vector<Vertex> fan_verts;

                while (!face_str.empty())
                {
                    std::size_t next_space = face_str.find(' ');
                    std::string_view group = (next_space == std::string_view::npos)
                                             ? face_str
                                             : face_str.substr(0, next_space);

                    if (!group.empty())
                    {
                        fan_verts.emplace_back();
                        Vertex& vtx = fan_verts.back();

                        for (int i = 0; i < 3 && !group.empty(); ++i)
                        {
                            std::size_t slash = group.find('/');
                            std::string_view part = group.substr(0, slash);

                            if (!part.empty())
                            {
                                int idx = std::stoi(std::string(part));
                                idx = (idx > 0) ? idx - 1
                                                : static_cast<int>(vertices.size()) + idx;
                                if (i == 0)      vtx.v  = idx;
                                else if (i == 1) vtx.vt = idx;
                                else             vtx.vn = idx;
                            }

                            if (slash == std::string_view::npos) break;
                            group.remove_prefix(slash + 1);
                        }
                    }

                    if (next_space == std::string_view::npos) break;
                    face_str.remove_prefix(next_space + 1);
                }

                for (std::size_t i = 2; i < fan_verts.size(); ++i)
                    faces.emplace_back(Face{{fan_verts[0], fan_verts[i - 1], fan_verts[i]}});
            }
        }
    }


    const std::vector<Vector3>& Model::get_vertices() const
    {
        return vertices;
    }

    const std::vector<Vector3>& Model::get_normals() const
    {
        return normals;
    }

    const std::vector<Vector3>& Model::get_uvs() const
    {
        return uvs;
    }

    const std::vector<Face> &Model::get_faces() const
    {
        return faces;
    }

    const Matrix &Model::get_transform() const
    {
        return transform;
    }

    Model &Model::set_transform(Matrix new_transform)
    {
        this->transform = new_transform;
        return *this;
    }
}