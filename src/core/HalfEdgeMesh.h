#pragma once

#include <vector>
#include <unordered_map>
#include "Mesh.h"

struct HEVertex {
    Vec3 p;
    int edge = -1;
    bool valid = true;
    bool boundary = false;
};

struct HEFace {
    int edge = -1;
    bool valid = true;
};

struct HEHalfEdge {
    int from = -1;
    int to = -1;
    int next = -1;
    int twin = -1;
    int face = -1;
    bool valid = true;
};

class HalfEdgeMesh {
public:
    std::vector<HEVertex> vertices;
    std::vector<HEFace> faces;
    std::vector<HEHalfEdge> halfedges;

    void build_from_mesh(const Mesh& mesh);
    bool checkConsistency(std::string* err) const;
    bool one_ring_ordered(int v, std::vector<int>* out_ring, bool* out_boundary) const;

private:
    void mark_boundaries();
};
