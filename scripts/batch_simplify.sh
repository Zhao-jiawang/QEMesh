#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INPUT_DIR="${ROOT_DIR}/input"
OUTPUT_DIR="${ROOT_DIR}/output"
TSV_PATH="${ROOT_DIR}/results.tsv"
RATIOS=(0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9)

if [[ ! -x "${ROOT_DIR}/build/mesh_simp" ]]; then
  echo "mesh_simp not found. Build first: cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j" >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"

shopt -s nullglob
obj_files=("${INPUT_DIR}"/*.obj)
if [[ ${#obj_files[@]} -eq 0 ]]; then
  echo "No .obj files found in ${INPUT_DIR}" >&2
  exit 1
fi

for obj in "${obj_files[@]}"; do
  base="$(basename "${obj}")"
  name="${base%.*}"
  for ratio in "${RATIOS[@]}"; do
    out_path="${OUTPUT_DIR}/${name}_r${ratio}.obj"
    "${ROOT_DIR}/build/mesh_simp" "${obj}" "${out_path}" "${ratio}" --tsv "${TSV_PATH}" --model_name "${name}"
  done
done

echo "Done. TSV: ${TSV_PATH}"
