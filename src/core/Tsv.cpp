#include "Tsv.h"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>

namespace {

std::string current_timestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    return std::string(buf);
}

bool needs_header(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        return true;
    }
    in.seekg(0, std::ios::end);
    return in.tellg() == 0;
}

} // namespace

bool append_tsv(const std::string& path,
                const std::string& model,
                double ratio,
                const SimplifyStats& stats,
                std::string* err) {
    bool write_header = needs_header(path);
    std::ofstream out(path, std::ios::app);
    if (!out) {
        if (err) {
            *err = "Failed to open TSV: " + path;
        }
        return false;
    }

    if (write_header) {
        out << "model\tratio\t"
            << "V_in\tF_in\tV_out\tF_out\t"
            << "time_total_ms\ttime_build_ms\ttime_simplify_ms\t"
            << "heap_push\theap_pop\tlocal_recompute_edges\tskipped_invalid\tnonmanifold_rejects\tconsistency_ok\t"
            << "qem_err_mean\tqem_err_max\t"
            << "degenerate_faces\taspect_mean\n";
    }

    out << model << "\t"
        << std::fixed << std::setprecision(6) << ratio << "\t"
        << stats.initial_vertices << "\t"
        << stats.initial_faces << "\t"
        << stats.final_vertices << "\t"
        << stats.final_faces << "\t"
        << std::fixed << std::setprecision(3) << stats.time_total_ms << "\t"
        << stats.time_build_ms << "\t"
        << stats.time_simplify_ms << "\t"
        << stats.heap_push << "\t"
        << stats.heap_pop << "\t"
        << stats.local_recompute_edges << "\t"
        << stats.skipped_invalid << "\t"
        << stats.nonmanifold_rejects << "\t"
        << stats.consistency_ok << "\t"
        << std::fixed << std::setprecision(8) << stats.qem_err_mean << "\t"
        << stats.qem_err_max << "\t"
        << stats.degenerate_faces << "\t"
        << std::fixed << std::setprecision(6) << stats.aspect_mean << "\n";

    return true;
}
