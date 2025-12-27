#include "HalfEdgeMesh.h"

#include <unordered_map>
#include <sstream>

namespace {

long long edge_key(int a, int b) {
    return (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
}

} // namespace

void HalfEdgeMesh::build_from_mesh(const Mesh& mesh) {
    vertices.clear();
    faces.clear();
    halfedges.clear();

    vertices.reserve(mesh.vertices.size());
    for (const auto& v : mesh.vertices) {
        HEVertex hv;
        hv.p = v.p;
        hv.valid = v.valid;
        vertices.push_back(hv);
    }

    std::unordered_map<long long, int> edge_map;

    for (size_t fi = 0; fi < mesh.faces.size(); ++fi) {
        const Face& f = mesh.faces[fi];
        if (!f.valid) {
            continue;
        }
        HEFace hf;
        hf.edge = static_cast<int>(halfedges.size());
        faces.push_back(hf);

        int base = static_cast<int>(halfedges.size());
        halfedges.resize(base + 3);
        for (int k = 0; k < 3; ++k) {
            int from = f.v[k];
            int to = f.v[(k + 1) % 3];
            HEHalfEdge& he = halfedges[base + k];
            he.from = from;
            he.to = to;
            he.face = static_cast<int>(faces.size() - 1);
            he.next = base + (k + 1) % 3;
            he.twin = -1;
            he.valid = true;

            if (vertices[from].edge == -1) {
                vertices[from].edge = base + k;
            }

            long long key = edge_key(from, to);
            edge_map[key] = base + k;
        }
    }

    for (const auto& kv : edge_map) {
        long long key = kv.first;
        int from = static_cast<int>(key >> 32);
        int to = static_cast<int>(key & 0xffffffff);
        long long rev = edge_key(to, from);
        auto it = edge_map.find(rev);
        if (it != edge_map.end()) {
            int e0 = kv.second;
            int e1 = it->second;
            halfedges[e0].twin = e1;
        }
    }

    mark_boundaries();
}

void HalfEdgeMesh::mark_boundaries() {
    for (auto& v : vertices) {
        v.boundary = false;
    }
    for (const auto& he : halfedges) {
        if (!he.valid) {
            continue;
        }
        if (he.twin == -1) {
            if (he.from >= 0 && he.from < static_cast<int>(vertices.size())) {
                vertices[he.from].boundary = true;
            }
            if (he.to >= 0 && he.to < static_cast<int>(vertices.size())) {
                vertices[he.to].boundary = true;
            }
        }
    }
}

bool HalfEdgeMesh::checkConsistency(std::string* err) const {
    for (size_t i = 0; i < halfedges.size(); ++i) {
        const HEHalfEdge& he = halfedges[i];
        if (!he.valid) {
            continue;
        }
        if (he.from < 0 || he.from >= static_cast<int>(vertices.size()) || !vertices[he.from].valid) {
            if (err) *err = "HalfEdge from invalid vertex.";
            return false;
        }
        if (he.to < 0 || he.to >= static_cast<int>(vertices.size()) || !vertices[he.to].valid) {
            if (err) *err = "HalfEdge to invalid vertex.";
            return false;
        }
        if (he.next < 0 || he.next >= static_cast<int>(halfedges.size())) {
            if (err) *err = "HalfEdge next invalid.";
            return false;
        }
        if (he.face < 0 || he.face >= static_cast<int>(faces.size())) {
            if (err) *err = "HalfEdge face invalid.";
            return false;
        }
        if (he.twin != -1) {
            if (he.twin < 0 || he.twin >= static_cast<int>(halfedges.size())) {
                if (err) *err = "HalfEdge twin invalid.";
                return false;
            }
            const HEHalfEdge& tw = halfedges[he.twin];
            if (tw.twin != static_cast<int>(i)) {
                if (err) *err = "HalfEdge twin mismatch.";
                return false;
            }
            if (tw.from != he.to || tw.to != he.from) {
                if (err) *err = "HalfEdge twin orientation mismatch.";
                return false;
            }
        }
    }
    return true;
}

bool HalfEdgeMesh::one_ring_ordered(int v, std::vector<int>* out_ring, bool* out_boundary) const {
    if (out_ring) {
        out_ring->clear();
    }
    if (out_boundary) {
        *out_boundary = false;
    }
    if (v < 0 || v >= static_cast<int>(vertices.size())) {
        return false;
    }
    if (!vertices[v].valid) {
        return false;
    }
    int start = vertices[v].edge;
    if (start < 0 || start >= static_cast<int>(halfedges.size())) {
        return false;
    }

    int he = start;
    std::unordered_map<int, bool> visited;
    for (;;) {
        if (visited[he]) {
            break;
        }
        visited[he] = true;
        const HEHalfEdge& e = halfedges[he];
        if (e.from != v) {
            return false;
        }
        if (out_ring) {
            out_ring->push_back(e.to);
        }
        if (e.twin == -1) {
            if (out_boundary) {
                *out_boundary = true;
            }
            break;
        }
        he = halfedges[e.twin].next;
        if (he == start) {
            break;
        }
    }

    return true;
}
