#pragma once

#include "Mesh.h"

struct SimplifyStats {
    int initial_vertices = 0;
    int initial_faces = 0;
    int final_vertices = 0;
    int final_faces = 0;
    int collapsed_edges = 0;
    int skipped_invalid = 0;
    int heap_push = 0;
    int heap_pop = 0;
    int local_recompute_edges = 0;
    int nonmanifold_rejects = 0;
    int consistency_ok = 0;
    double time_total_ms = 0.0;
    double time_build_ms = 0.0;
    double time_simplify_ms = 0.0;
    double qem_err_mean = 0.0;
    double qem_err_max = 0.0;
    int degenerate_faces = 0;
    double aspect_mean = 0.0;
};

enum class SimplifyMethod {
    QEM,
    ShortestEdge,
    Custom
};

enum class SimplifyStrategy {
    EdgeCollapse,
    VertexDelete,
    FaceContract
};

struct SimplifyOptions {
    SimplifyMethod method = SimplifyMethod::QEM;
    SimplifyStrategy strategy = SimplifyStrategy::EdgeCollapse;
    double lambda = 0.0;
};

bool simplify_mesh(Mesh& mesh, double ratio, const SimplifyOptions& options, SimplifyStats* stats, std::string* err);
