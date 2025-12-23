#include "Simplify.h"

#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <chrono>

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

bool compute_edge_collapse(const Mesh& mesh, int v1, int v2, Vec3* out_pos, double* out_cost) {
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

    auto eval_cost = [&](const Vec3& p) {
        return q.evaluate(p);
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
    if (invert3x3(m, inv)) {
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

} // namespace

bool simplify_mesh(Mesh& mesh, double ratio, SimplifyStats* stats, std::string* err) {
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
                if (!compute_edge_collapse(mesh, static_cast<int>(v), n, &pos, &cost)) {
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
        if (!compute_edge_collapse(mesh, e.v1, e.v2, &pos, &cost)) {
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
            if (!compute_edge_collapse(mesh, e.v1, n, &npos, &ncost)) {
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
