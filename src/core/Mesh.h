#pragma once

#include <string>
#include <vector>
#include <unordered_set>
#include "Vec3.h"
#include "Quadric.h"

struct Vertex {
    Vec3 p;
    Quadric q;
    bool valid = true;
    bool boundary = false;
    int version = 0;
};

struct Face {
    int v[3] = {0, 0, 0};
    bool valid = true;
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    std::vector<std::unordered_set<int>> vfaces;
    std::vector<std::unordered_set<int>> vneighbors;

    bool load_obj(const std::string& path, std::string* err);
    bool save_obj(const std::string& path, std::string* err) const;

    void build_adjacency();
    void compute_quadrics();
    int valid_face_count() const;
    int valid_vertex_count() const;

    bool are_neighbors(int a, int b) const;
    bool face_has_edge(int face_id, int a, int b) const;
    void update_boundary(int vid);
    bool checkConsistency(std::string* err) const;
    int degenerate_face_count(double eps) const;
    double aspect_mean(double eps) const;

    void compact(std::vector<int>* out_vmap = nullptr);
};

Vec3 face_normal(const Mesh& mesh, const Face& f);
