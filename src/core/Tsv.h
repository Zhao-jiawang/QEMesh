#pragma once

#include <string>
#include "Simplify.h"

bool append_tsv(const std::string& path,
                const std::string& model,
                const std::string& method,
                const std::string& strategy,
                double ratio,
                const SimplifyStats& stats,
                std::string* err);
