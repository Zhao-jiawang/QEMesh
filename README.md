# QEM Mesh Simplifier

## Dependencies (Ubuntu/Debian)
```
sudo apt-get install -y build-essential cmake xorg-dev libgl1-mesa-dev
```

## Build
```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run
```
./build/mesh_simp input.obj output.obj 0.3 --tsv results.tsv --model_name bunny
./build/mesh_viewer input.obj output.obj
```

## Viewer Shortcuts
- W: wireframe
- S: simplify
- O: reload output
- R: reset camera
- B: toggle before/after

## TSV Output
- Appended per simplification run to `results.tsv` (or `--tsv` path)
- Fields: model,ratio,V_in,F_in,V_out,F_out,time_total_ms,time_build_ms,time_simplify_ms,heap_push,heap_pop,local_recompute_edges,skipped_invalid,nonmanifold_rejects,consistency_ok,qem_err_mean,qem_err_max,degenerate_faces,aspect_mean
