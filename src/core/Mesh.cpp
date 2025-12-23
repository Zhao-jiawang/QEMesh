#include "Mesh.h"

#include <fstream>
#include <sstream>
#include <unordered_map>
#include <algorithm>
#include <cctype>

namespace {

int parse_index(const std::string& token, int vcount) {
    size_t slash = token.find('/');
    std::string part = (slash == std::string::npos) ? token : token.substr(0, slash);
    if (part.empty()) {
        return -1;
    }
    int idx = std::stoi(part);
    if (idx < 0) {
        idx = vcount + idx + 1;
    }
    return idx - 1;
}

} // namespace

bool Mesh::load_obj(const std::string& path, std::string* err) {
    std::ifstream in(path);
    if (!in) {
        if (err) {
            *err = "Failed to open input OBJ: " + path;
        }
        return false;
    }

    vertices.clear();
    faces.clear();

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        if (line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v") {
            double x, y, z;
            ss >> x >> y >> z;
            vertices.push_back(Vertex{Vec3(x, y, z)});
        } else if (tag == "f") {
            std::vector<int> idx;
            std::string tok;
            while (ss >> tok) {
                int vi = parse_index(tok, static_cast<int>(vertices.size()));
                if (vi < 0 || vi >= static_cast<int>(vertices.size())) {
                    if (err) {
                        *err = "OBJ face index out of range.";
                    }
                    return false;
                }
                idx.push_back(vi);
            }
            if (idx.size() < 3) {
                continue;
            }
            for (size_t i = 1; i + 1 < idx.size(); ++i) {
                Face f;
                f.v[0] = idx[0];
                f.v[1] = idx[i];
                f.v[2] = idx[i + 1];
                faces.push_back(f);
            }
        }
    }

    build_adjacency();
    compute_quadrics();
    return true;
}

bool Mesh::save_obj(const std::string& path, std::string* err) const {
    std::ofstream out(path);
    if (!out) {
        if (err) {
            *err = "Failed to write output OBJ: " + path;
        }
        return false;
    }

    std::vector<int> vmap(vertices.size(), -1);
    int new_idx = 0;
    for (size_t i = 0; i < vertices.size(); ++i) {
        if (vertices[i].valid) {
            vmap[i] = new_idx++;
            out << "v " << vertices[i].p.x << " " << vertices[i].p.y << " " << vertices[i].p.z << "\n";
        }
    }

    for (const auto& f : faces) {
        if (!f.valid) {
            continue;
        }
        int a = vmap[f.v[0]];
        int b = vmap[f.v[1]];
        int c = vmap[f.v[2]];
        if (a < 0 || b < 0 || c < 0) {
            continue;
        }
        out << "f " << (a + 1) << " " << (b + 1) << " " << (c + 1) << "\n";
    }
    return true;
}

void Mesh::build_adjacency() {
    vfaces.assign(vertices.size(), {});
    vneighbors.assign(vertices.size(), {});

    std::unordered_map<long long, int> edge_count;
    auto edge_key = [](int a, int b) {
        if (a > b) std::swap(a, b);
        return (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
    };

    for (size_t fi = 0; fi < faces.size(); ++fi) {
        Face& f = faces[fi];
        f.valid = true;
        for (int k = 0; k < 3; ++k) {
            vfaces[f.v[k]].insert(static_cast<int>(fi));
        }
        for (int e = 0; e < 3; ++e) {
            int a = f.v[e];
            int b = f.v[(e + 1) % 3];
            vneighbors[a].insert(b);
            vneighbors[b].insert(a);
            edge_count[edge_key(a, b)] += 1;
        }
    }

    for (size_t i = 0; i < vertices.size(); ++i) {
        vertices[i].boundary = false;
    }
    for (const auto& kv : edge_count) {
        if (kv.second == 1) {
            int a = static_cast<int>(kv.first >> 32);
            int b = static_cast<int>(kv.first & 0xffffffff);
            vertices[a].boundary = true;
            vertices[b].boundary = true;
        }
    }
}

void Mesh::compute_quadrics() {
    for (auto& v : vertices) {
        v.q = Quadric();
    }
    for (const auto& f : faces) {
        if (!f.valid) {
            continue;
        }
        Vec3 n = face_normal(*this, f);
        double len = length(n);
        if (len <= 1e-12) {
            continue;
        }
        n = n / len;
        double d = -dot(n, vertices[f.v[0]].p);
        Quadric k = Quadric::from_plane(n, d);
        vertices[f.v[0]].q += k;
        vertices[f.v[1]].q += k;
        vertices[f.v[2]].q += k;
    }
}

int Mesh::valid_face_count() const {
    int count = 0;
    for (const auto& f : faces) {
        if (f.valid) {
            ++count;
        }
    }
    return count;
}

int Mesh::valid_vertex_count() const {
    int count = 0;
    for (const auto& v : vertices) {
        if (v.valid) {
            ++count;
        }
    }
    return count;
}

bool Mesh::are_neighbors(int a, int b) const {
    if (a < 0 || b < 0 || a >= static_cast<int>(vneighbors.size()) || b >= static_cast<int>(vneighbors.size())) {
        return false;
    }
    return vneighbors[a].find(b) != vneighbors[a].end();
}

bool Mesh::face_has_edge(int face_id, int a, int b) const {
    const Face& f = faces[face_id];
    bool has_a = false, has_b = false;
    for (int k = 0; k < 3; ++k) {
        has_a = has_a || (f.v[k] == a);
        has_b = has_b || (f.v[k] == b);
    }
    return has_a && has_b;
}

void Mesh::update_boundary(int vid) {
    if (!vertices[vid].valid) {
        vertices[vid].boundary = false;
        return;
    }
    bool is_boundary = false;
    for (int n : vneighbors[vid]) {
        if (!vertices[n].valid) {
            continue;
        }
        int count = 0;
        for (int f_id : vfaces[vid]) {
            if (!faces[f_id].valid) {
                continue;
            }
            if (face_has_edge(f_id, vid, n)) {
                ++count;
            }
        }
        if (count == 1) {
            is_boundary = true;
            break;
        }
    }
    vertices[vid].boundary = is_boundary;
}

bool Mesh::checkConsistency(std::string* err) const {
    for (size_t fi = 0; fi < faces.size(); ++fi) {
        const Face& f = faces[fi];
        if (!f.valid) {
            continue;
        }
        for (int k = 0; k < 3; ++k) {
            int v = f.v[k];
            if (v < 0 || v >= static_cast<int>(vertices.size()) || !vertices[v].valid) {
                if (err) {
                    *err = "Face references invalid vertex.";
                }
                return false;
            }
            if (vfaces[v].find(static_cast<int>(fi)) == vfaces[v].end()) {
                if (err) {
                    *err = "Face-vertex adjacency mismatch.";
                }
                return false;
            }
        }
    }

    for (size_t v = 0; v < vertices.size(); ++v) {
        if (!vertices[v].valid) {
            continue;
        }
        for (int n : vneighbors[v]) {
            if (n < 0 || n >= static_cast<int>(vertices.size()) || !vertices[n].valid) {
                if (err) {
                    *err = "Neighbor references invalid vertex.";
                }
                return false;
            }
            bool found = false;
            for (int fid : vfaces[v]) {
                if (!faces[fid].valid) {
                    continue;
                }
                if (face_has_edge(fid, static_cast<int>(v), n)) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                if (err) {
                    *err = "Neighbor without shared face.";
                }
                return false;
            }
        }
    }
    return true;
}

int Mesh::degenerate_face_count(double eps) const {
    int count = 0;
    for (const auto& f : faces) {
        if (!f.valid) {
            continue;
        }
        Vec3 n = face_normal(*this, f);
        double area2 = length(n) * 0.5;
        if (area2 < eps) {
            ++count;
        }
    }
    return count;
}

double Mesh::aspect_mean(double eps) const {
    double sum = 0.0;
    int count = 0;
    for (const auto& f : faces) {
        if (!f.valid) {
            continue;
        }
        const Vec3& a = vertices[f.v[0]].p;
        const Vec3& b = vertices[f.v[1]].p;
        const Vec3& c = vertices[f.v[2]].p;
        double ab = length(b - a);
        double bc = length(c - b);
        double ca = length(a - c);
        double max_e = std::max(ab, std::max(bc, ca));
        Vec3 n = cross(b - a, c - a);
        double area = 0.5 * length(n);
        if (area < eps) {
            continue;
        }
        double aspect = (max_e * max_e) / (2.0 * area);
        sum += aspect;
        count++;
    }
    if (count == 0) {
        return 0.0;
    }
    return sum / static_cast<double>(count);
}

void Mesh::compact(std::vector<int>* out_vmap) {
    std::vector<int> vmap(vertices.size(), -1);
    std::vector<Vertex> new_vertices;
    new_vertices.reserve(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        if (vertices[i].valid) {
            vmap[i] = static_cast<int>(new_vertices.size());
            new_vertices.push_back(vertices[i]);
        }
    }

    std::vector<Face> new_faces;
    new_faces.reserve(faces.size());
    for (const auto& f : faces) {
        if (!f.valid) {
            continue;
        }
        Face nf;
        nf.v[0] = vmap[f.v[0]];
        nf.v[1] = vmap[f.v[1]];
        nf.v[2] = vmap[f.v[2]];
        if (nf.v[0] < 0 || nf.v[1] < 0 || nf.v[2] < 0) {
            continue;
        }
        new_faces.push_back(nf);
    }
    vertices.swap(new_vertices);
    faces.swap(new_faces);
    if (out_vmap) {
        *out_vmap = std::move(vmap);
    }
    build_adjacency();
    compute_quadrics();
}

Vec3 face_normal(const Mesh& mesh, const Face& f) {
    const Vec3& a = mesh.vertices[f.v[0]].p;
    const Vec3& b = mesh.vertices[f.v[1]].p;
    const Vec3& c = mesh.vertices[f.v[2]].p;
    return cross(b - a, c - a);
}
