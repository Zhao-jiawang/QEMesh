#include <cstdio>
#include <string>
#include <cstdlib>

#include "../core/Mesh.h"
#include "../core/Simplify.h"
#include "../core/Tsv.h"

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "Usage: mesh_simp in.obj out.obj ratio [--tsv results.tsv] [--model_name name] [--seed 123]\n");
        return 1;
    }
    std::string input;
    std::string output;
    double ratio = 0.0;
    bool ratio_set = false;
    std::string csv_path = "results.tsv";
    std::string model_name;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--tsv" && i + 1 < argc) {
            csv_path = argv[++i];
        } else if (arg == "--model_name" && i + 1 < argc) {
            model_name = argv[++i];
        } else if (arg == "--seed" && i + 1 < argc) {
            unsigned int seed = static_cast<unsigned int>(std::stoul(argv[++i]));
            std::srand(seed);
        } else if (!arg.empty() && arg[0] == '-') {
            std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
            return 1;
        } else {
            if (input.empty()) {
                input = arg;
            } else if (output.empty()) {
                output = arg;
            } else if (!ratio_set) {
                ratio = std::stod(arg);
                ratio_set = true;
            } else {
                std::fprintf(stderr, "Too many positional arguments.\n");
                return 1;
            }
        }
    }

    if (input.empty() || output.empty() || !ratio_set) {
        std::fprintf(stderr, "Usage: mesh_simp in.obj out.obj ratio [--tsv results.tsv] [--model_name name] [--seed 123]\n");
        return 1;
    }
    if (model_name.empty()) {
        std::string base = input;
        size_t slash = base.find_last_of("/\\");
        if (slash != std::string::npos) {
            base = base.substr(slash + 1);
        }
        size_t dot = base.find_last_of('.');
        if (dot != std::string::npos) {
            base = base.substr(0, dot);
        }
        model_name = base;
    }

    Mesh mesh;
    std::string err;
    if (!mesh.load_obj(input, &err)) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }

    SimplifyStats stats;
    if (!simplify_mesh(mesh, ratio, &stats, &err)) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }

    if (!mesh.save_obj(output, &err)) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }

    if (!append_tsv(csv_path, model_name, ratio, stats, &err)) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }

    std::printf("Done. V/F %d/%d -> %d/%d, collapsed %d, skipped %d. TSV: %s\n",
        stats.initial_vertices, stats.initial_faces,
        stats.final_vertices, stats.final_faces,
        stats.collapsed_edges, stats.skipped_invalid, csv_path.c_str());
    return 0;
}
