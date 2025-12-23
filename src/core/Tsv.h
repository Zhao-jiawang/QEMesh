#pragma once

#include <string>
#include "Simplify.h"

bool append_tsv(const std::string& path,
                const std::string& model,
                double ratio,
                const SimplifyStats& stats,
                std::string* err);
