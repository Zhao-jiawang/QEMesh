#include "Simplify.h"
#include "HalfEdgeMesh.h"

#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>

namespace {

struct EdgeEntry {
    double cost = 0.0;
    int v1 = -1;
    int v2 = -1;
    Vec3 target;
    int v1_version = 0;
    int v2_version = 0;
};

struct EdgeCompare {
    bool operator()(const EdgeEntry& a, const EdgeEntry& b) const {
        return a.cost > b.cost;
    }
};

struct VertexEntry {
    double cost = 0.0;
    int v = -1;
    int version = 0;
};

struct VertexCompare {
    bool operator()(const VertexEntry& a, const VertexEntry& b) const {
        return a.cost > b.cost;
    }
};

struct FaceEntry {
    double cost = 0.0;
    int face = -1;
    int v0_version = 0;
    int v1_version = 0;
    int v2_version = 0;
    Vec3 target;
};

struct FaceCompare {
    bool operator()(const FaceEntry& a, const FaceEntry& b) const {
        return a.cost > b.cost;
    }
};

bool invert3x3(const double m[3][3], double inv[3][3]) {
    double det =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (std::fabs(det) < 1e-12) {
        return false;
    }
    double inv_det = 1.0 / det;
    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;
    return true;
}

double edge_length_sq(const Mesh& mesh, int v1, int v2) {
    const Vec3& a = mesh.vertices[v1].p;
    const Vec3& b = mesh.vertices[v2].p;
    Vec3 d = b - a;
    return dot(d, d);
}

bool compute_edge_collapse(const Mesh& mesh, int v1, int v2, SimplifyMethod method, double lambda, Vec3* out_pos, double* out_cost) {
    const Vertex& a = mesh.vertices[v1];
    const Vertex& b = mesh.vertices[v2];
    if (!a.valid || !b.valid) {
        return false;
    }
    if (a.boundary != b.boundary) {
        return false;
    }

    Quadric q = a.q;
    q += b.q;
    double len_sq = edge_length_sq(mesh, v1, v2);

    auto eval_cost = [&](const Vec3& p) {
        if (method == SimplifyMethod::ShortestEdge) {
            (void)p;
            return len_sq;
        }
        double qem = q.evaluate(p);
        if (method == SimplifyMethod::Custom) {
            return qem + lambda * len_sq;
        }
        return qem;
    };

    if (a.boundary && b.boundary) {
        double ca = eval_cost(a.p);
        double cb = eval_cost(b.p);
        if (ca <= cb) {
            *out_pos = a.p;
            *out_cost = ca;
        } else {
            *out_pos = b.p;
            *out_cost = cb;
        }
        return true;
    }

    double m[3][3] = {
        {q.m[0][0], q.m[0][1], q.m[0][2]},
        {q.m[1][0], q.m[1][1], q.m[1][2]},
        {q.m[2][0], q.m[2][1], q.m[2][2]},
    };
    double inv[3][3];
    if (method != SimplifyMethod::ShortestEdge && invert3x3(m, inv)) {
        double rhs[3] = {-q.m[0][3], -q.m[1][3], -q.m[2][3]};
        Vec3 v(
            inv[0][0] * rhs[0] + inv[0][1] * rhs[1] + inv[0][2] * rhs[2],
            inv[1][0] * rhs[0] + inv[1][1] * rhs[1] + inv[1][2] * rhs[2],
            inv[2][0] * rhs[0] + inv[2][1] * rhs[1] + inv[2][2] * rhs[2]
        );
        *out_pos = v;
        *out_cost = eval_cost(v);
        return true;
    }

    Vec3 mid = (a.p + b.p) * 0.5;
    if (method == SimplifyMethod::ShortestEdge) {
        *out_pos = mid;
        *out_cost = len_sq;
        return true;
    }

    Vec3 candidates[3] = {a.p, b.p, mid};
    double best_cost = eval_cost(candidates[0]);
    Vec3 best_pos = candidates[0];
    for (int i = 1; i < 3; ++i) {
        double c = eval_cost(candidates[i]);
        if (c < best_cost) {
            best_cost = c;
            best_pos = candidates[i];
        }
    }
    *out_pos = best_pos;
    *out_cost = best_cost;
    return true;
}

bool face_contains(const Face& f, int v) {
    return f.v[0] == v || f.v[1] == v || f.v[2] == v;
}

bool face_contains_both(const Face& f, int a, int b) {
    bool has_a = false;
    bool has_b = false;
    for (int k = 0; k < 3; ++k) {
        has_a = has_a || (f.v[k] == a);
        has_b = has_b || (f.v[k] == b);
    }
    return has_a && has_b;
}

bool face_duplicate_after(const Mesh& mesh, const Face& f, int v_keep, int v_remove) {
    int nv[3] = {f.v[0], f.v[1], f.v[2]};
    for (int k = 0; k < 3; ++k) {
        if (nv[k] == v_remove) {
            nv[k] = v_keep;
        }
    }
    int a = nv[0], b = nv[1], c = nv[2];
    if (a == b || b == c || a == c) {
        return false;
    }
    int sorted[3] = {a, b, c};
    std::sort(sorted, sorted + 3);

    int min_v = sorted[0];
    if (min_v < 0 || min_v >= static_cast<int>(mesh.vfaces.size())) {
        return false;
    }
    for (int face_id : mesh.vfaces[min_v]) {
        const Face& other = mesh.faces[face_id];
        if (!other.valid) {
            continue;
        }
        int ov[3] = {other.v[0], other.v[1], other.v[2]};
        std::sort(ov, ov + 3);
        if (ov[0] == sorted[0] && ov[1] == sorted[1] && ov[2] == sorted[2]) {
            if (&other != &f) {
                return true;
            }
        }
    }
    return false;
}

bool face_exists(const Mesh& mesh, int a, int b, int c, int exclude_fid = -1) {
    if (a == b || b == c || a == c) {
        return true;
    }
    int sorted[3] = {a, b, c};
    std::sort(sorted, sorted + 3);
    int min_v = sorted[0];
    if (min_v < 0 || min_v >= static_cast<int>(mesh.vfaces.size())) {
        return false;
    }
    for (int face_id : mesh.vfaces[min_v]) {
        if (face_id == exclude_fid) {
            continue;
        }
        const Face& other = mesh.faces[face_id];
        if (!other.valid) {
            continue;
        }
        int ov[3] = {other.v[0], other.v[1], other.v[2]};
        std::sort(ov, ov + 3);
        if (ov[0] == sorted[0] && ov[1] == sorted[1] && ov[2] == sorted[2]) {
            return true;
        }
    }
    return false;
}

bool find_duplicate_face(const Mesh& mesh, int a, int b, int c, int exclude_fid, int* out_fid) {
    if (a == b || b == c || a == c) {
        return false;
    }
    int sorted[3] = {a, b, c};
    std::sort(sorted, sorted + 3);
    int min_v = sorted[0];
    if (min_v < 0 || min_v >= static_cast<int>(mesh.vfaces.size())) {
        return false;
    }
    for (int face_id : mesh.vfaces[min_v]) {
        if (face_id == exclude_fid) {
            continue;
        }
        const Face& other = mesh.faces[face_id];
        if (!other.valid) {
            continue;
        }
        int ov[3] = {other.v[0], other.v[1], other.v[2]};
        std::sort(ov, ov + 3);
        if (ov[0] == sorted[0] && ov[1] == sorted[1] && ov[2] == sorted[2]) {
            if (out_fid) {
                *out_fid = face_id;
            }
            return true;
        }
    }
    return false;
}

std::string face_key(int a, int b, int c) {
    int v[3] = {a, b, c};
    std::sort(v, v + 3);
    return std::to_string(v[0]) + "_" + std::to_string(v[1]) + "_" + std::to_string(v[2]);
}

int edge_face_count(const Mesh& mesh, int a, int b) {
    int count = 0;
    if (a < 0 || b < 0 || a >= static_cast<int>(mesh.vfaces.size())) {
        return 0;
    }
    for (int fid : mesh.vfaces[a]) {
        if (!mesh.faces[fid].valid) {
            continue;
        }
        if (mesh.face_has_edge(fid, a, b)) {
            ++count;
        }
    }
    return count;
}

int edge_face_count_excluding(const Mesh& mesh, int a, int b, int exclude_v) {
    int count = 0;
    if (a < 0 || b < 0 || a >= static_cast<int>(mesh.vfaces.size())) {
        return 0;
    }
    for (int fid : mesh.vfaces[a]) {
        if (!mesh.faces[fid].valid) {
            continue;
        }
        const Face& f = mesh.faces[fid];
        if (face_contains(f, exclude_v)) {
            continue;
        }
        if (mesh.face_has_edge(fid, a, b)) {
            ++count;
        }
    }
    return count;
}

void orthonormal_basis(const Vec3& n, Vec3* out_u, Vec3* out_v) {
    Vec3 nrm = normalize(n);
    Vec3 a = (std::fabs(nrm.x) < 0.9) ? Vec3(1.0, 0.0, 0.0) : Vec3(0.0, 1.0, 0.0);
    Vec3 u = normalize(cross(nrm, a));
    Vec3 v = cross(nrm, u);
    *out_u = u;
    *out_v = v;
}

double area2_2d(const Vec3& a, const Vec3& b, const Vec3& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool point_in_tri_2d(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& p) {
    double a1 = area2_2d(a, b, p);
    double a2 = area2_2d(b, c, p);
    double a3 = area2_2d(c, a, p);
    bool has_neg = (a1 < 0.0) || (a2 < 0.0) || (a3 < 0.0);
    bool has_pos = (a1 > 0.0) || (a2 > 0.0) || (a3 > 0.0);
    return !(has_neg && has_pos);
}

bool triangulate_polygon(const Mesh& mesh,
                         const std::vector<int>& ring,
                         const Vec3& normal,
                         std::vector<Face>* out_tris) {
    if (ring.size() < 3) {
        return false;
    }
    Vec3 u, v;
    orthonormal_basis(normal, &u, &v);
    std::vector<Vec3> proj(ring.size());
    for (size_t i = 0; i < ring.size(); ++i) {
        const Vec3& p = mesh.vertices[ring[i]].p;
        proj[i] = Vec3(dot(p, u), dot(p, v), 0.0);
    }
    double area = 0.0;
    for (size_t i = 0; i < proj.size(); ++i) {
        const Vec3& a = proj[i];
        const Vec3& b = proj[(i + 1) % proj.size()];
        area += (a.x * b.y - b.x * a.y);
    }
    std::vector<int> poly = ring;
    if (area < 0.0) {
        std::reverse(poly.begin(), poly.end());
        std::reverse(proj.begin(), proj.end());
    }

    std::vector<int> index(poly.size());
    for (size_t i = 0; i < poly.size(); ++i) {
        index[i] = static_cast<int>(i);
    }

    out_tris->clear();
    const double eps = 1e-12;
    int guard = 0;
    while (index.size() >= 3 && guard < 10000) {
        guard++;
        bool ear_found = false;
        for (size_t i = 0; i < index.size(); ++i) {
            int i0 = index[(i + index.size() - 1) % index.size()];
            int i1 = index[i];
            int i2 = index[(i + 1) % index.size()];
            Vec3 a2 = proj[i0];
            Vec3 b2 = proj[i1];
            Vec3 c2 = proj[i2];
            double a2d = area2_2d(a2, b2, c2);
            if (a2d <= eps) {
                continue;
            }
            bool has_inside = false;
            for (size_t j = 0; j < index.size(); ++j) {
                int ij = index[j];
                if (ij == i0 || ij == i1 || ij == i2) {
                    continue;
                }
                if (point_in_tri_2d(a2, b2, c2, proj[ij])) {
                    has_inside = true;
                    break;
                }
            }
            if (has_inside) {
                continue;
            }
            Face tri;
            tri.v[0] = poly[i0];
            tri.v[1] = poly[i1];
            tri.v[2] = poly[i2];
            Vec3 n = normalize(face_normal(mesh, tri));
            if (length(n) <= eps || dot(n, normal) < 1e-8) {
                continue;
            }
            out_tris->push_back(tri);
            index.erase(index.begin() + static_cast<long>(i));
            ear_found = true;
            break;
        }
        if (!ear_found) {
            out_tris->clear();
            return false;
        }
    }
    return !out_tris->empty();
}

bool order_ring_by_angle(const Mesh& mesh, int center, std::vector<int>* out_ring) {
    if (!out_ring) {
        return false;
    }
    out_ring->clear();
    if (center < 0 || center >= static_cast<int>(mesh.vertices.size())) {
        return false;
    }
    if (!mesh.vertices[center].valid) {
        return false;
    }
    Vec3 normal(0.0, 0.0, 0.0);
    for (int fid : mesh.vfaces[center]) {
        if (!mesh.faces[fid].valid) {
            continue;
        }
        normal += normalize(face_normal(mesh, mesh.faces[fid]));
    }
    if (length(normal) <= 1e-12) {
        return false;
    }
    normal = normalize(normal);
    Vec3 u, v;
    orthonormal_basis(normal, &u, &v);
    std::vector<std::pair<double, int>> angles;
    angles.reserve(mesh.vneighbors[center].size());
    const Vec3& c = mesh.vertices[center].p;
    for (int n : mesh.vneighbors[center]) {
        if (!mesh.vertices[n].valid) {
            continue;
        }
        Vec3 d = mesh.vertices[n].p - c;
        double x = dot(d, u);
        double y = dot(d, v);
        double ang = std::atan2(y, x);
        angles.push_back({ang, n});
    }
    if (angles.size() < 3) {
        return false;
    }
    std::sort(angles.begin(), angles.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    for (const auto& kv : angles) {
        out_ring->push_back(kv.second);
    }
    return true;
}

bool can_collapse(const Mesh& mesh, int v1, int v2, const Vec3& new_pos, bool* out_nonmanifold) {
    std::unordered_set<int> affected;
    affected.insert(mesh.vfaces[v1].begin(), mesh.vfaces[v1].end());
    affected.insert(mesh.vfaces[v2].begin(), mesh.vfaces[v2].end());
    std::unordered_map<int, int> edge_use;
    if (out_nonmanifold) {
        *out_nonmanifold = false;
    }

    for (int fid : affected) {
        const Face& f = mesh.faces[fid];
        if (!f.valid) {
            continue;
        }
        if (face_contains_both(f, v1, v2)) {
            continue;
        }

        if (face_duplicate_after(mesh, f, v1, v2)) {
            return false;
        }

        int nv[3] = {f.v[0], f.v[1], f.v[2]};
        for (int k = 0; k < 3; ++k) {
            if (nv[k] == v2) {
                nv[k] = v1;
            }
        }
        if (nv[0] == nv[1] || nv[1] == nv[2] || nv[0] == nv[2]) {
            return false;
        }
        Vec3 p[3] = {
            (nv[0] == v1) ? new_pos : mesh.vertices[nv[0]].p,
            (nv[1] == v1) ? new_pos : mesh.vertices[nv[1]].p,
            (nv[2] == v1) ? new_pos : mesh.vertices[nv[2]].p
        };
        Vec3 old_n = normalize(face_normal(mesh, f));
        Vec3 new_n = normalize(cross(p[1] - p[0], p[2] - p[0]));
        if (length(new_n) <= 1e-12) {
            return false;
        }
        double dot_n = dot(old_n, new_n);
        if (dot_n < 1e-8) {
            return false;
        }

        for (int e = 0; e < 3; ++e) {
            int a = nv[e];
            int b = nv[(e + 1) % 3];
            if (a == v1 || b == v1) {
                int n = (a == v1) ? b : a;
                edge_use[n] += 1;
                if (edge_use[n] > 2) {
                    if (out_nonmanifold) {
                        *out_nonmanifold = true;
                    }
                    return false;
                }
            }
        }
    }
    return true;
}

void remove_face(Mesh& mesh, int fid) {
    if (!mesh.faces[fid].valid) {
        return;
    }
    Face& f = mesh.faces[fid];
    f.valid = false;
    for (int k = 0; k < 3; ++k) {
        mesh.vfaces[f.v[k]].erase(fid);
    }
}

void cleanup_neighbors(Mesh& mesh, int vid) {
    std::vector<int> to_remove;
    for (int n : mesh.vneighbors[vid]) {
        if (!mesh.vertices[n].valid) {
            to_remove.push_back(n);
            continue;
        }
        int count = 0;
        for (int f_id : mesh.vfaces[vid]) {
            if (!mesh.faces[f_id].valid) {
                continue;
            }
            if (mesh.face_has_edge(f_id, vid, n)) {
                ++count;
            }
        }
        if (count == 0) {
            to_remove.push_back(n);
        }
    }
    for (int n : to_remove) {
        mesh.vneighbors[vid].erase(n);
        mesh.vneighbors[n].erase(vid);
    }
}

int collapse_edge(Mesh& mesh, int v1, int v2, const Vec3& new_pos) {
    Vertex& keep = mesh.vertices[v1];
    Vertex& rem = mesh.vertices[v2];
    keep.p = new_pos;
    keep.q += rem.q;
    rem.valid = false;

    int removed_faces = 0;
    std::unordered_set<int> affected;
    affected.insert(mesh.vfaces[v1].begin(), mesh.vfaces[v1].end());
    affected.insert(mesh.vfaces[v2].begin(), mesh.vfaces[v2].end());

    for (int fid : affected) {
        Face& f = mesh.faces[fid];
        if (!f.valid) {
            continue;
        }
        if (face_contains_both(f, v1, v2)) {
            remove_face(mesh, fid);
            removed_faces++;
            continue;
        }
        bool changed = false;
        for (int k = 0; k < 3; ++k) {
            if (f.v[k] == v2) {
                f.v[k] = v1;
                changed = true;
            }
        }
        if (changed) {
            mesh.vfaces[v2].erase(fid);
            mesh.vfaces[v1].insert(fid);
        }
        if (f.v[0] == f.v[1] || f.v[1] == f.v[2] || f.v[0] == f.v[2]) {
            remove_face(mesh, fid);
            removed_faces++;
        }
    }

    for (int n : mesh.vneighbors[v2]) {
        mesh.vneighbors[n].erase(v2);
        if (n != v1) {
            mesh.vneighbors[n].insert(v1);
            mesh.vneighbors[v1].insert(n);
        }
    }
    mesh.vneighbors[v1].erase(v2);
    mesh.vneighbors[v2].clear();
    mesh.vfaces[v2].clear();

    return removed_faces;
}

void add_face(Mesh& mesh, const Face& f) {
    int fid = static_cast<int>(mesh.faces.size());
    mesh.faces.push_back(f);
    for (int k = 0; k < 3; ++k) {
        int v = f.v[k];
        mesh.vfaces[v].insert(fid);
    }
    for (int e = 0; e < 3; ++e) {
        int a = f.v[e];
        int b = f.v[(e + 1) % 3];
        mesh.vneighbors[a].insert(b);
        mesh.vneighbors[b].insert(a);
    }
}

bool triangulate_fan(const Mesh& mesh, const std::vector<int>& ring, const Vec3& normal, std::vector<Face>* out_tris) {
    out_tris->clear();
    if (ring.size() < 3) {
        return false;
    }
    const double eps = 1e-12;
    for (size_t i = 1; i + 1 < ring.size(); ++i) {
        Face tri;
        tri.v[0] = ring[0];
        tri.v[1] = ring[i];
        tri.v[2] = ring[i + 1];
        Vec3 n = normalize(face_normal(mesh, tri));
        if (length(n) <= eps || dot(n, normal) < 1e-8) {
            return false;
        }
        out_tris->push_back(tri);
    }
    return !out_tris->empty();
}

bool compute_vertex_delete_cost(const Mesh& mesh, int v, SimplifyMethod method, double lambda, double* out_cost) {
    if (v < 0 || v >= static_cast<int>(mesh.vertices.size())) {
        return false;
    }
    if (!mesh.vertices[v].valid || mesh.vertices[v].boundary) {
        return false;
    }
    double best = std::numeric_limits<double>::infinity();
    for (int n : mesh.vneighbors[v]) {
        Vec3 pos;
        double cost;
        if (!compute_edge_collapse(mesh, v, n, method, lambda, &pos, &cost)) {
            continue;
        }
        if (cost < best) {
            best = cost;
        }
    }
    if (!std::isfinite(best)) {
        return false;
    }
    *out_cost = best;
    return true;
}

bool delete_vertex(Mesh& mesh, int v, std::vector<int>* affected_vertices, int* removed_faces, int* added_faces) {
    HalfEdgeMesh hem;
    std::vector<int> ring;
    bool boundary = false;
    if (mesh.vneighbors[v].size() <= 200) {
        hem.build_from_mesh(mesh);
        if (!hem.one_ring_ordered(v, &ring, &boundary)) {
            boundary = mesh.vertices[v].boundary;
            ring.clear();
            if (!order_ring_by_angle(mesh, v, &ring)) {
                return false;
            }
        }
    } else {
        boundary = mesh.vertices[v].boundary;
        if (!order_ring_by_angle(mesh, v, &ring)) {
            return false;
        }
    }
    if (boundary || ring.size() < 3) {
        return false;
    }

    Vec3 normal(0.0, 0.0, 0.0);
    for (int fid : mesh.vfaces[v]) {
        if (!mesh.faces[fid].valid) {
            continue;
        }
        Vec3 n = normalize(face_normal(mesh, mesh.faces[fid]));
        normal += n;
    }
    if (length(normal) <= 1e-12) {
        return false;
    }
    normal = normalize(normal);

    std::vector<Face> new_tris;
    bool use_ear = ring.size() <= 200;
    if (use_ear && triangulate_polygon(mesh, ring, normal, &new_tris)) {
    } else if (!triangulate_fan(mesh, ring, normal, &new_tris)) {
        return false;
    }

    for (const auto& tri : new_tris) {
        int dup_fid = -1;
        if (find_duplicate_face(mesh, tri.v[0], tri.v[1], tri.v[2], -1, &dup_fid)) {
            if (dup_fid >= 0) {
                const Face& dup_face = mesh.faces[dup_fid];
                if (!face_contains(dup_face, v)) {
                    return false;
                }
            } else {
                return false;
            }
        }
        int edges[3][2] = {{tri.v[0], tri.v[1]}, {tri.v[1], tri.v[2]}, {tri.v[2], tri.v[0]}};
        for (int e = 0; e < 3; ++e) {
            if (edge_face_count_excluding(mesh, edges[e][0], edges[e][1], v) >= 2) {
                return false;
            }
        }
    }

    std::unordered_set<int> to_remove = mesh.vfaces[v];
    if (removed_faces) {
        *removed_faces = static_cast<int>(to_remove.size());
    }
    for (int fid : to_remove) {
        remove_face(mesh, fid);
    }

    mesh.vertices[v].valid = false;
    for (int n : mesh.vneighbors[v]) {
        mesh.vneighbors[n].erase(v);
    }
    mesh.vneighbors[v].clear();
    mesh.vfaces[v].clear();

    for (const auto& tri : new_tris) {
        add_face(mesh, tri);
    }
    if (added_faces) {
        *added_faces = static_cast<int>(new_tris.size());
    }

    if (affected_vertices) {
        affected_vertices->assign(ring.begin(), ring.end());
    }
    return true;
}

bool compute_face_contract(const Mesh& mesh,
                           int fid,
                           SimplifyMethod method,
                           double lambda,
                           Vec3* out_pos,
                           double* out_cost) {
    const Face& f = mesh.faces[fid];
    const Vertex& v0 = mesh.vertices[f.v[0]];
    const Vertex& v1 = mesh.vertices[f.v[1]];
    const Vertex& v2 = mesh.vertices[f.v[2]];
    if (!v0.valid || !v1.valid || !v2.valid) {
        return false;
    }

    Quadric q = v0.q;
    q += v1.q;
    q += v2.q;

    double len_sq = edge_length_sq(mesh, f.v[0], f.v[1]) +
                    edge_length_sq(mesh, f.v[1], f.v[2]) +
                    edge_length_sq(mesh, f.v[2], f.v[0]);

    auto eval_cost = [&](const Vec3& p) {
        if (method == SimplifyMethod::ShortestEdge) {
            (void)p;
            return len_sq;
        }
        double qem = q.evaluate(p);
        if (method == SimplifyMethod::Custom) {
            return qem + lambda * len_sq;
        }
        return qem;
    };

    Vec3 centroid = (v0.p + v1.p + v2.p) / 3.0;
    double m[3][3] = {
        {q.m[0][0], q.m[0][1], q.m[0][2]},
        {q.m[1][0], q.m[1][1], q.m[1][2]},
        {q.m[2][0], q.m[2][1], q.m[2][2]},
    };
    double inv[3][3];
    bool has_boundary = v0.boundary || v1.boundary || v2.boundary;
    if (!has_boundary && method != SimplifyMethod::ShortestEdge && invert3x3(m, inv)) {
        double rhs[3] = {-q.m[0][3], -q.m[1][3], -q.m[2][3]};
        Vec3 v(
            inv[0][0] * rhs[0] + inv[0][1] * rhs[1] + inv[0][2] * rhs[2],
            inv[1][0] * rhs[0] + inv[1][1] * rhs[1] + inv[1][2] * rhs[2],
            inv[2][0] * rhs[0] + inv[2][1] * rhs[1] + inv[2][2] * rhs[2]
        );
        *out_pos = v;
        *out_cost = eval_cost(v);
        return true;
    }
    if (has_boundary) {
        Vec3 candidates[4] = {v0.p, v1.p, v2.p, centroid};
        double best = eval_cost(candidates[0]);
        Vec3 best_pos = candidates[0];
        for (int i = 1; i < 4; ++i) {
            double c = eval_cost(candidates[i]);
            if (c < best) {
                best = c;
                best_pos = candidates[i];
            }
        }
        *out_pos = best_pos;
        *out_cost = best;
        return true;
    }
    *out_pos = centroid;
    *out_cost = eval_cost(centroid);
    return true;
}

bool estimate_contract_face(const Mesh& mesh, int fid, int keep, const Vec3& new_pos, int* out_removed, bool* out_nonmanifold) {
    const Face& f = mesh.faces[fid];
    int rem1 = f.v[0];
    int rem2 = f.v[1];
    if (keep == f.v[0]) {
        rem1 = f.v[1];
        rem2 = f.v[2];
    } else if (keep == f.v[1]) {
        rem1 = f.v[0];
        rem2 = f.v[2];
    } else {
        rem1 = f.v[0];
        rem2 = f.v[1];
    }
    if (out_nonmanifold) {
        *out_nonmanifold = false;
    }
    if (out_removed) {
        *out_removed = 0;
    }
    std::unordered_set<int> affected;
    affected.insert(mesh.vfaces[keep].begin(), mesh.vfaces[keep].end());
    affected.insert(mesh.vfaces[rem1].begin(), mesh.vfaces[rem1].end());
    affected.insert(mesh.vfaces[rem2].begin(), mesh.vfaces[rem2].end());

    std::unordered_map<int, int> edge_use;
    std::unordered_map<std::string, int> local_faces;
    int removed = 0;
    for (int fid2 : affected) {
        const Face& face = mesh.faces[fid2];
        if (!face.valid) {
            continue;
        }
        if (fid2 == fid) {
            removed++;
            continue;
        }
        int nv[3] = {face.v[0], face.v[1], face.v[2]};
        for (int k = 0; k < 3; ++k) {
            if (nv[k] == rem1 || nv[k] == rem2) {
                nv[k] = keep;
            }
        }
        if (nv[0] == nv[1] || nv[1] == nv[2] || nv[0] == nv[2]) {
            removed++;
            continue;
        }
        Face tmp;
        tmp.v[0] = nv[0];
        tmp.v[1] = nv[1];
        tmp.v[2] = nv[2];
        int dup_fid = -1;
        if (find_duplicate_face(mesh, tmp.v[0], tmp.v[1], tmp.v[2], fid, &dup_fid)) {
            bool dup_in_affected = (dup_fid >= 0) && (affected.find(dup_fid) != affected.end());
            if (!dup_in_affected) {
                return false;
            }
        }

        Vec3 p[3] = {
            (nv[0] == keep) ? new_pos : mesh.vertices[nv[0]].p,
            (nv[1] == keep) ? new_pos : mesh.vertices[nv[1]].p,
            (nv[2] == keep) ? new_pos : mesh.vertices[nv[2]].p
        };
        Vec3 old_n = normalize(face_normal(mesh, face));
        Vec3 new_n = normalize(cross(p[1] - p[0], p[2] - p[0]));
        if (length(new_n) <= 1e-12) {
            return false;
        }
        if (dot(old_n, new_n) < 1e-8) {
            return false;
        }
        std::string key = face_key(tmp.v[0], tmp.v[1], tmp.v[2]);
        local_faces[key] += 1;
        for (int e = 0; e < 3; ++e) {
            int a = nv[e];
            int b = nv[(e + 1) % 3];
            if (a == keep || b == keep) {
                int n = (a == keep) ? b : a;
                edge_use[n] += 1;
                if (edge_use[n] > 2) {
                    if (out_nonmanifold) {
                        *out_nonmanifold = true;
                    }
                    return false;
                }
            }
        }
    }
    for (const auto& kv : local_faces) {
        if (kv.second > 1) {
            removed += (kv.second - 1);
        }
    }
    if (out_removed) {
        *out_removed = removed;
    }
    return true;
}

bool can_contract_face(const Mesh& mesh, int fid, int keep, const Vec3& new_pos, bool* out_nonmanifold) {
    return estimate_contract_face(mesh, fid, keep, new_pos, nullptr, out_nonmanifold);
}

bool contract_face(Mesh& mesh, int fid, int keep, const Vec3& new_pos, std::vector<int>* affected_vertices, int* removed_faces) {
    Face f = mesh.faces[fid];
    int rem1 = f.v[0];
    int rem2 = f.v[1];
    if (keep == f.v[0]) {
        rem1 = f.v[1];
        rem2 = f.v[2];
    } else if (keep == f.v[1]) {
        rem1 = f.v[0];
        rem2 = f.v[2];
    } else {
        rem1 = f.v[0];
        rem2 = f.v[1];
    }

    mesh.vertices[keep].p = new_pos;
    mesh.vertices[keep].q += mesh.vertices[rem1].q;
    mesh.vertices[keep].q += mesh.vertices[rem2].q;
    mesh.vertices[rem1].valid = false;
    mesh.vertices[rem2].valid = false;

    std::unordered_set<int> affected;
    affected.insert(mesh.vfaces[keep].begin(), mesh.vfaces[keep].end());
    affected.insert(mesh.vfaces[rem1].begin(), mesh.vfaces[rem1].end());
    affected.insert(mesh.vfaces[rem2].begin(), mesh.vfaces[rem2].end());

    int removed = 0;
    for (int fid2 : affected) {
        Face& face = mesh.faces[fid2];
        if (!face.valid) {
            continue;
        }
        if (fid2 == fid) {
            remove_face(mesh, fid2);
            removed++;
            continue;
        }
        bool changed = false;
        for (int k = 0; k < 3; ++k) {
            if (face.v[k] == rem1 || face.v[k] == rem2) {
                face.v[k] = keep;
                changed = true;
            }
        }
        if (changed) {
            mesh.vfaces[rem1].erase(fid2);
            mesh.vfaces[rem2].erase(fid2);
            mesh.vfaces[keep].insert(fid2);
        }
        if (face.v[0] == face.v[1] || face.v[1] == face.v[2] || face.v[0] == face.v[2]) {
            remove_face(mesh, fid2);
            removed++;
        }
    }

    std::unordered_map<std::string, int> seen;
    for (int fid2 : affected) {
        Face& face = mesh.faces[fid2];
        if (!face.valid) {
            continue;
        }
        std::string key = face_key(face.v[0], face.v[1], face.v[2]);
        auto it = seen.find(key);
        if (it == seen.end()) {
            seen.emplace(key, fid2);
        } else {
            remove_face(mesh, fid2);
            removed++;
        }
    }

    for (int n : mesh.vneighbors[rem1]) {
        mesh.vneighbors[n].erase(rem1);
        if (n != keep) {
            mesh.vneighbors[keep].insert(n);
            mesh.vneighbors[n].insert(keep);
        }
    }
    for (int n : mesh.vneighbors[rem2]) {
        mesh.vneighbors[n].erase(rem2);
        if (n != keep) {
            mesh.vneighbors[keep].insert(n);
            mesh.vneighbors[n].insert(keep);
        }
    }
    mesh.vneighbors[rem1].clear();
    mesh.vneighbors[rem2].clear();
    mesh.vfaces[rem1].clear();
    mesh.vfaces[rem2].clear();

    if (removed_faces) {
        *removed_faces = removed;
    }
    if (affected_vertices) {
        affected_vertices->clear();
        affected_vertices->push_back(keep);
        for (int n : mesh.vneighbors[keep]) {
            affected_vertices->push_back(n);
        }
    }
    return true;
}

bool simplify_vertex_delete(Mesh& mesh, double ratio, const SimplifyOptions& options, SimplifyStats* stats, std::string* err) {
    if (ratio <= 0.0 || ratio > 1.0) {
        if (err) {
            *err = "ratio must be in (0,1].";
        }
        return false;
    }
    if (stats) {
        *stats = SimplifyStats();
    }

    auto total_start = std::chrono::high_resolution_clock::now();
    auto build_start = std::chrono::high_resolution_clock::now();
    mesh.build_adjacency();
    mesh.compute_quadrics();

    int initial_faces = mesh.valid_face_count();
    if (initial_faces == 0) {
        if (err) {
            *err = "Mesh has no faces.";
        }
        return false;
    }
    if (stats) {
        stats->initial_vertices = mesh.valid_vertex_count();
        stats->initial_faces = initial_faces;
    }

    int target_faces = static_cast<int>(std::ceil(initial_faces * ratio));
    int current_faces = initial_faces;

    std::priority_queue<VertexEntry, std::vector<VertexEntry>, VertexCompare> heap;
    for (size_t v = 0; v < mesh.vertices.size(); ++v) {
        if (!mesh.vertices[v].valid || mesh.vertices[v].boundary) {
            continue;
        }
        double cost = 0.0;
        if (!compute_vertex_delete_cost(mesh, static_cast<int>(v), options.method, options.lambda, &cost)) {
            continue;
        }
        VertexEntry e;
        e.v = static_cast<int>(v);
        e.cost = cost;
        e.version = mesh.vertices[e.v].version;
        heap.push(e);
        if (stats) {
            stats->heap_push++;
        }
    }

    auto build_end = std::chrono::high_resolution_clock::now();
    if (stats) {
        stats->time_build_ms =
            std::chrono::duration<double, std::milli>(build_end - build_start).count();
    }

    int skipped_invalid = 0;
    int collapsed = 0;
    double cost_sum = 0.0;
    double cost_max = 0.0;
    int cost_count = 0;
    auto simplify_start = std::chrono::high_resolution_clock::now();

    while (current_faces > target_faces && !heap.empty()) {
        VertexEntry e = heap.top();
        heap.pop();
        if (stats) {
            stats->heap_pop++;
        }
        if (e.v < 0 || e.v >= static_cast<int>(mesh.vertices.size())) {
            continue;
        }
        if (!mesh.vertices[e.v].valid) {
            continue;
        }
        if (mesh.vertices[e.v].version != e.version) {
            continue;
        }
        if (mesh.vertices[e.v].boundary) {
            continue;
        }

        std::vector<int> affected;
        int removed_faces = 0;
        int added_faces = 0;
        if (!delete_vertex(mesh, e.v, &affected, &removed_faces, &added_faces)) {
            skipped_invalid++;
            continue;
        }
        current_faces = current_faces - removed_faces + added_faces;
        mesh.vertices[e.v].version++;
        for (int v : affected) {
            mesh.vertices[v].version++;
        }

        for (int v : affected) {
            cleanup_neighbors(mesh, v);
            mesh.update_boundary(v);
        }

        for (int v : affected) {
            if (!mesh.vertices[v].valid || mesh.vertices[v].boundary) {
                continue;
            }
            double cost = 0.0;
            if (!compute_vertex_delete_cost(mesh, v, options.method, options.lambda, &cost)) {
                continue;
            }
            VertexEntry ne;
            ne.v = v;
            ne.cost = cost;
            ne.version = mesh.vertices[v].version;
            heap.push(ne);
            if (stats) {
                stats->heap_push++;
                stats->local_recompute_edges++;
            }
        }

        cost_sum += e.cost;
        cost_max = std::max(cost_max, e.cost);
        cost_count++;
        collapsed++;
    }

    auto simplify_end = std::chrono::high_resolution_clock::now();
    if (stats) {
        stats->time_simplify_ms =
            std::chrono::duration<double, std::milli>(simplify_end - simplify_start).count();
    }

    mesh.compact();
    double qem_mean = (cost_count > 0) ? (cost_sum / static_cast<double>(cost_count)) : 0.0;

    const double eps = 1e-12;
    int degenerate_faces = mesh.degenerate_face_count(eps);
    double aspect_mean = mesh.aspect_mean(eps);

    auto total_end = std::chrono::high_resolution_clock::now();
    double elapsed_total = std::chrono::duration<double, std::milli>(total_end - total_start).count();

    if (stats) {
        stats->final_vertices = mesh.valid_vertex_count();
        stats->final_faces = mesh.valid_face_count();
        stats->collapsed_edges = collapsed;
        stats->skipped_invalid = skipped_invalid;
        stats->time_total_ms = elapsed_total;
        stats->qem_err_mean = qem_mean;
        stats->qem_err_max = cost_max;
        stats->degenerate_faces = degenerate_faces;
        stats->aspect_mean = aspect_mean;
        stats->consistency_ok = mesh.checkConsistency(nullptr) ? 1 : 0;
    }

    return true;
}

bool simplify_face_contract(Mesh& mesh, double ratio, const SimplifyOptions& options, SimplifyStats* stats, std::string* err) {
    if (ratio <= 0.0 || ratio > 1.0) {
        if (err) {
            *err = "ratio must be in (0,1].";
        }
        return false;
    }
    if (stats) {
        *stats = SimplifyStats();
    }

    auto total_start = std::chrono::high_resolution_clock::now();
    auto build_start = std::chrono::high_resolution_clock::now();
    mesh.build_adjacency();
    mesh.compute_quadrics();

    int initial_faces = mesh.valid_face_count();
    if (initial_faces == 0) {
        if (err) {
            *err = "Mesh has no faces.";
        }
        return false;
    }
    if (stats) {
        stats->initial_vertices = mesh.valid_vertex_count();
        stats->initial_faces = initial_faces;
    }

    int target_faces = static_cast<int>(std::ceil(initial_faces * ratio));
    int current_faces = initial_faces;

    std::priority_queue<FaceEntry, std::vector<FaceEntry>, FaceCompare> heap;
    for (size_t fid = 0; fid < mesh.faces.size(); ++fid) {
        if (!mesh.faces[fid].valid) {
            continue;
        }
        Vec3 pos;
        double cost;
        if (!compute_face_contract(mesh, static_cast<int>(fid), options.method, options.lambda, &pos, &cost)) {
            continue;
        }
        FaceEntry e;
        e.face = static_cast<int>(fid);
        e.cost = cost;
        e.target = pos;
        e.v0_version = mesh.vertices[mesh.faces[fid].v[0]].version;
        e.v1_version = mesh.vertices[mesh.faces[fid].v[1]].version;
        e.v2_version = mesh.vertices[mesh.faces[fid].v[2]].version;
        heap.push(e);
        if (stats) {
            stats->heap_push++;
        }
    }

    auto build_end = std::chrono::high_resolution_clock::now();
    if (stats) {
        stats->time_build_ms =
            std::chrono::duration<double, std::milli>(build_end - build_start).count();
    }

    int skipped_invalid = 0;
    int collapsed = 0;
    double cost_sum = 0.0;
    double cost_max = 0.0;
    int cost_count = 0;
    auto simplify_start = std::chrono::high_resolution_clock::now();

    while (current_faces > target_faces && !heap.empty()) {
        FaceEntry e = heap.top();
        heap.pop();
        if (stats) {
            stats->heap_pop++;
        }
        if (e.face < 0 || e.face >= static_cast<int>(mesh.faces.size())) {
            continue;
        }
        if (!mesh.faces[e.face].valid) {
            continue;
        }
        Face f = mesh.faces[e.face];
        if (mesh.vertices[f.v[0]].version != e.v0_version ||
            mesh.vertices[f.v[1]].version != e.v1_version ||
            mesh.vertices[f.v[2]].version != e.v2_version) {
            continue;
        }

        Vec3 pos;
        double cost;
        if (!compute_face_contract(mesh, e.face, options.method, options.lambda, &pos, &cost)) {
            skipped_invalid++;
            continue;
        }
        int slack = current_faces - target_faces;
        bool nonmanifold = false;
        bool contracted = false;
        bool defer = false;
        FaceEntry best_entry = e;
        int best_keep = -1;
        Vec3 best_pos = pos;
        int best_removed = std::numeric_limits<int>::max();
        std::vector<int> candidates = {f.v[0], f.v[1], f.v[2]};
        std::sort(candidates.begin(), candidates.end(), [&](int a, int b) {
            return mesh.vertices[a].boundary < mesh.vertices[b].boundary;
        });
        for (int keep : candidates) {
            int removed_est = 0;
            if (!estimate_contract_face(mesh, e.face, keep, pos, &removed_est, &nonmanifold)) {
                continue;
            }
            if (slack > 0 && removed_est > slack) {
                if (slack <= 10) {
                    if (removed_est < best_removed) {
                        best_removed = removed_est;
                        best_keep = keep;
                        best_pos = pos;
                    }
                    defer = true;
                    continue;
                }
            }
            std::vector<int> affected;
            int removed_faces = 0;
            contract_face(mesh, e.face, keep, pos, &affected, &removed_faces);
            current_faces -= removed_faces;
            for (int v : affected) {
                mesh.vertices[v].version++;
            }

            for (int v : affected) {
                cleanup_neighbors(mesh, v);
                mesh.update_boundary(v);
            }

            for (int v : affected) {
                for (int fid : mesh.vfaces[v]) {
                    if (!mesh.faces[fid].valid) {
                        continue;
                    }
                    Vec3 npos;
                    double ncost;
                    if (!compute_face_contract(mesh, fid, options.method, options.lambda, &npos, &ncost)) {
                        continue;
                    }
                    FaceEntry ne;
                    ne.face = fid;
                    ne.cost = ncost;
                    ne.target = npos;
                    ne.v0_version = mesh.vertices[mesh.faces[fid].v[0]].version;
                    ne.v1_version = mesh.vertices[mesh.faces[fid].v[1]].version;
                    ne.v2_version = mesh.vertices[mesh.faces[fid].v[2]].version;
                    heap.push(ne);
                    if (stats) {
                        stats->heap_push++;
                        stats->local_recompute_edges++;
                    }
                }
            }

            cost_sum += cost;
            cost_max = std::max(cost_max, cost);
            cost_count++;
            collapsed++;
            contracted = true;
            break;
        }

        if (!contracted && defer && best_keep >= 0) {
            std::vector<int> affected;
            int removed_faces = 0;
            contract_face(mesh, e.face, best_keep, best_pos, &affected, &removed_faces);
            current_faces -= removed_faces;
            for (int v : affected) {
                mesh.vertices[v].version++;
            }
            for (int v : affected) {
                cleanup_neighbors(mesh, v);
                mesh.update_boundary(v);
            }
            for (int v : affected) {
                for (int fid : mesh.vfaces[v]) {
                    if (!mesh.faces[fid].valid) {
                        continue;
                    }
                    Vec3 npos;
                    double ncost;
                    if (!compute_face_contract(mesh, fid, options.method, options.lambda, &npos, &ncost)) {
                        continue;
                    }
                    FaceEntry ne;
                    ne.face = fid;
                    ne.cost = ncost;
                    ne.target = npos;
                    ne.v0_version = mesh.vertices[mesh.faces[fid].v[0]].version;
                    ne.v1_version = mesh.vertices[mesh.faces[fid].v[1]].version;
                    ne.v2_version = mesh.vertices[mesh.faces[fid].v[2]].version;
                    heap.push(ne);
                    if (stats) {
                        stats->heap_push++;
                        stats->local_recompute_edges++;
                    }
                }
            }
            cost_sum += cost;
            cost_max = std::max(cost_max, cost);
            cost_count++;
            collapsed++;
            contracted = true;
        }

        if (!contracted) {
            skipped_invalid++;
            if (nonmanifold && stats) {
                stats->nonmanifold_rejects++;
            }
            continue;
        }
    }

    auto simplify_end = std::chrono::high_resolution_clock::now();
    if (stats) {
        stats->time_simplify_ms =
            std::chrono::duration<double, std::milli>(simplify_end - simplify_start).count();
    }

    mesh.compact();
    double qem_mean = (cost_count > 0) ? (cost_sum / static_cast<double>(cost_count)) : 0.0;

    const double eps = 1e-12;
    int degenerate_faces = mesh.degenerate_face_count(eps);
    double aspect_mean = mesh.aspect_mean(eps);

    auto total_end = std::chrono::high_resolution_clock::now();
    double elapsed_total = std::chrono::duration<double, std::milli>(total_end - total_start).count();

    if (stats) {
        stats->final_vertices = mesh.valid_vertex_count();
        stats->final_faces = mesh.valid_face_count();
        stats->collapsed_edges = collapsed;
        stats->skipped_invalid = skipped_invalid;
        stats->time_total_ms = elapsed_total;
        stats->qem_err_mean = qem_mean;
        stats->qem_err_max = cost_max;
        stats->degenerate_faces = degenerate_faces;
        stats->aspect_mean = aspect_mean;
        stats->consistency_ok = mesh.checkConsistency(nullptr) ? 1 : 0;
    }

    return true;
}

} // namespace

bool simplify_edge_collapse(Mesh& mesh, double ratio, const SimplifyOptions& options, SimplifyStats* stats, std::string* err) {
    if (ratio <= 0.0 || ratio > 1.0) {
        if (err) {
            *err = "ratio must be in (0,1].";
        }
        return false;
    }

    if (stats) {
        *stats = SimplifyStats();
    }

    auto total_start = std::chrono::high_resolution_clock::now();

    auto build_start = std::chrono::high_resolution_clock::now();
    mesh.build_adjacency();
    mesh.compute_quadrics();

    int initial_faces = mesh.valid_face_count();
    if (initial_faces == 0) {
        if (err) {
            *err = "Mesh has no faces.";
        }
        return false;
    }

    if (stats) {
        stats->initial_vertices = mesh.valid_vertex_count();
        stats->initial_faces = initial_faces;
    }

    int target_faces = static_cast<int>(std::ceil(initial_faces * ratio));
    int current_faces = initial_faces;

    std::priority_queue<EdgeEntry, std::vector<EdgeEntry>, EdgeCompare> heap;
    for (size_t v = 0; v < mesh.vertices.size(); ++v) {
        if (!mesh.vertices[v].valid) {
            continue;
        }
        for (int n : mesh.vneighbors[v]) {
            if (static_cast<int>(v) < n) {
                Vec3 pos;
                double cost;
                if (!compute_edge_collapse(mesh, static_cast<int>(v), n, options.method, options.lambda, &pos, &cost)) {
                    continue;
                }
                EdgeEntry e;
                e.v1 = static_cast<int>(v);
                e.v2 = n;
                e.target = pos;
                e.cost = cost;
                e.v1_version = mesh.vertices[e.v1].version;
                e.v2_version = mesh.vertices[e.v2].version;
                heap.push(e);
                if (stats) {
                    stats->heap_push++;
                }
            }
        }
    }

    auto build_end = std::chrono::high_resolution_clock::now();
    if (stats) {
        stats->time_build_ms =
            std::chrono::duration<double, std::milli>(build_end - build_start).count();
    }

    int skipped_invalid = 0;
    int collapsed_edges = 0;
    double qem_sum = 0.0;
    double qem_max = 0.0;
    int qem_count = 0;
    auto simplify_start = std::chrono::high_resolution_clock::now();

    while (current_faces > target_faces && !heap.empty()) {
        EdgeEntry e = heap.top();
        heap.pop();
        if (stats) {
            stats->heap_pop++;
        }

        if (e.v1 < 0 || e.v2 < 0) {
            continue;
        }
        if (!mesh.vertices[e.v1].valid || !mesh.vertices[e.v2].valid) {
            continue;
        }
        if (mesh.vertices[e.v1].version != e.v1_version || mesh.vertices[e.v2].version != e.v2_version) {
            continue;
        }
        if (!mesh.are_neighbors(e.v1, e.v2)) {
            continue;
        }

        Vec3 pos;
        double cost;
        if (!compute_edge_collapse(mesh, e.v1, e.v2, options.method, options.lambda, &pos, &cost)) {
            skipped_invalid++;
            continue;
        }
        bool nonmanifold = false;
        if (!can_collapse(mesh, e.v1, e.v2, pos, &nonmanifold)) {
            skipped_invalid++;
            if (nonmanifold && stats) {
                stats->nonmanifold_rejects++;
            }
            continue;
        }

        int removed_faces = collapse_edge(mesh, e.v1, e.v2, pos);
        current_faces -= removed_faces;
        mesh.vertices[e.v1].version++;
        qem_sum += cost;
        qem_max = std::max(qem_max, cost);
        qem_count++;

        std::unordered_set<int> update_vertices;
        update_vertices.insert(e.v1);
        for (int n : mesh.vneighbors[e.v1]) {
            update_vertices.insert(n);
        }
        for (int vid : update_vertices) {
            cleanup_neighbors(mesh, vid);
        }
        for (int vid : update_vertices) {
            mesh.update_boundary(vid);
        }

        for (int n : mesh.vneighbors[e.v1]) {
            Vec3 npos;
            double ncost;
            if (!compute_edge_collapse(mesh, e.v1, n, options.method, options.lambda, &npos, &ncost)) {
                continue;
            }
            EdgeEntry ne;
            ne.v1 = std::min(e.v1, n);
            ne.v2 = std::max(e.v1, n);
            ne.target = npos;
            ne.cost = ncost;
            ne.v1_version = mesh.vertices[ne.v1].version;
            ne.v2_version = mesh.vertices[ne.v2].version;
            heap.push(ne);
            if (stats) {
                stats->heap_push++;
                stats->local_recompute_edges++;
            }
        }

        collapsed_edges++;
    }

    auto simplify_end = std::chrono::high_resolution_clock::now();
    if (stats) {
        stats->time_simplify_ms =
            std::chrono::duration<double, std::milli>(simplify_end - simplify_start).count();
    }

    mesh.compact();
    if (stats) {
        stats->consistency_ok = mesh.checkConsistency(nullptr) ? 1 : 0;
    }

    double qem_mean = (qem_count > 0) ? (qem_sum / static_cast<double>(qem_count)) : 0.0;

    const double eps = 1e-12;
    int degenerate_faces = mesh.degenerate_face_count(eps);
    double aspect_mean = mesh.aspect_mean(eps);

    auto total_end = std::chrono::high_resolution_clock::now();
    double elapsed_total = std::chrono::duration<double, std::milli>(total_end - total_start).count();

    if (stats) {
        stats->final_vertices = mesh.valid_vertex_count();
        stats->final_faces = mesh.valid_face_count();
        stats->collapsed_edges = collapsed_edges;
        stats->skipped_invalid = skipped_invalid;
        stats->time_total_ms = elapsed_total;
        stats->qem_err_mean = qem_mean;
        stats->qem_err_max = qem_max;
        stats->degenerate_faces = degenerate_faces;
        stats->aspect_mean = aspect_mean;
    }

    return true;
}

bool simplify_mesh(Mesh& mesh, double ratio, const SimplifyOptions& options, SimplifyStats* stats, std::string* err) {
    switch (options.strategy) {
        case SimplifyStrategy::VertexDelete:
            return simplify_vertex_delete(mesh, ratio, options, stats, err);
        case SimplifyStrategy::FaceContract:
            return simplify_face_contract(mesh, ratio, options, stats, err);
        case SimplifyStrategy::EdgeCollapse:
        default:
            return simplify_edge_collapse(mesh, ratio, options, stats, err);
    }
}
