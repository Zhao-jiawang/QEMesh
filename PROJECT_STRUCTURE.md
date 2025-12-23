# QEM Mesh Simplifier 项目结构说明（详细版）

本文档更详细地说明每个源文件包含的关键结构体/变量/函数、它们的职责，以及文件之间的调用关系。

## 目录结构

```
.
├── CMakeLists.txt
├── README.md
├── PROJECT_STRUCTURE.md
├── scripts/
│   └── batch_simplify.sh
├── src/
│   ├── core/
│   │   ├── Vec3.h
│   │   ├── Quadric.h
│   │   ├── Mesh.h
│   │   ├── Mesh.cpp
│   │   ├── Simplify.h
│   │   ├── Simplify.cpp
│   │   ├── Tsv.h
│   │   └── Tsv.cpp
│   ├── cli/
│   │   └── main.cpp
│   └── viewer/
│       ├── Math.h
│       ├── Shader.h
│       ├── Shader.cpp
│       └── main.cpp
├── input/             # 输入 OBJ
├── output/            # 简化后 OBJ 输出
└── results.tsv        # 批量或单次运行的统计结果
```

## 文件详细解读与函数/变量说明

### CMakeLists.txt
- 作用：构建入口。
- 关键目标：
  - `mesh_core` 静态库（核心算法）。
  - `mesh_simp` CLI 可执行。
  - `mesh_viewer` OpenGL 可执行。
- 依赖：GLFW/GLAD/ImGui 通过 FetchContent 拉取。
- 调用关系：
  - `mesh_simp` / `mesh_viewer` 均链接 `mesh_core`。

### README.md
- 作用：最简运行说明。
- 包含：构建命令、运行命令、viewer 快捷键、TSV 指标说明。

### scripts/batch_simplify.sh
- 作用：批量简化脚本。
- 变量：
  - `ROOT_DIR`/`INPUT_DIR`/`OUTPUT_DIR`/`TSV_PATH`/`RATIOS`。
- 核心逻辑：
  - 扫描 `input/*.obj`，对每个模型遍历 0.1..0.9，调用 `build/mesh_simp`。
- 调用关系：直接调用 `mesh_simp`，产生 `results.tsv` 与 `output/*.obj`。

### src/core/Vec3.h
- 结构体：`Vec3`
  - 成员变量：`x, y, z`。
  - 运算：`operator + - * /`、`operator +=`。
- 函数：
  - `dot(a,b)`：点积。
  - `cross(a,b)`：叉积。
  - `length(v)`：向量长度。
  - `normalize(v)`：归一化（含零长度保护）。
- 调用关系：几乎所有几何计算都依赖该头文件。

### src/core/Quadric.h
- 结构体：`Quadric`
  - 成员变量：`m[4][4]`，4x4 二次误差矩阵。
  - 成员函数：
    - 构造函数：矩阵清零。
    - `operator+=`：累加 quadric。
    - `from_plane(n, d)`：构造平面 quadric（K=p*p^T）。
    - `evaluate(v)`：计算 `v_hat^T Q v_hat`。
- 调用关系：
  - `Mesh::compute_quadrics()` 构建顶点 quadric。
  - `Simplify.cpp` 中用 quadric 计算收缩代价。

### src/core/Mesh.h / Mesh.cpp
- 结构体：
  - `Vertex`：成员 `p`(位置), `q`(Quadric), `valid`, `boundary`, `version`。
  - `Face`：成员 `v[3]`, `valid`。
- 类：`Mesh`
  - 主要成员：
    - `vertices` / `faces`：顶点与三角面数组。
    - `vfaces`：顶点关联面集合（unordered_set<int>）。
    - `vneighbors`：顶点一环邻居集合。
  - 主要函数：
    - `load_obj(path, err)`：读取 OBJ（v/f），多边形 fan 三角化。
    - `save_obj(path, err)`：输出有效顶点与三角面。
    - `build_adjacency()`：构建 `vfaces` / `vneighbors`，并标记边界顶点。
    - `compute_quadrics()`：对每个面计算 K，并累加到顶点 Q。
    - `valid_face_count()` / `valid_vertex_count()`：统计有效数量。
    - `are_neighbors(a,b)`：邻居查询。
    - `face_has_edge(face_id,a,b)`：判断面是否包含边。
    - `update_boundary(vid)`：更新单顶点边界标记。
    - `compact()`：移除无效顶点/面，重建索引与邻接。
    - `checkConsistency(err)`：基础一致性校验（面引用、邻居合法且共享面）。
    - `degenerate_face_count(eps)`：统计面积过小的面。
    - `aspect_mean(eps)`：三角形质量指标均值。
  - 辅助函数：
    - `face_normal(mesh, face)`：面法线向量。
- 调用关系：
  - `Simplify.cpp` 依赖 Mesh 的邻接、quadric、边界与一致性检查。
  - Viewer/CLI 通过 Mesh 读写 OBJ。

### src/core/Simplify.h / Simplify.cpp
- 结构体：`SimplifyStats`
  - 包含：输入/输出规模、耗时、heap 统计、QEM 误差、质量指标、拓扑统计。
- 主要对外函数：
  - `simplify_mesh(mesh, ratio, stats, err)`。
- Simplify.cpp 内部关键结构与函数：
  - `EdgeEntry`：堆节点（cost, v1, v2, target, ver1, ver2）。
  - `EdgeCompare`：小顶堆比较器。
  - `invert3x3()`：求 3x3 逆（用于求最优点）。
  - `compute_edge_collapse()`：
    - 构造 Q = Q1 + Q2。
    - 解最优点（A*v=b）。若不可逆回退 {v1,v2,mid}。
    - 边界点对：强制收缩到代价更小端点。
  - `can_collapse()`：
    - 拒绝重复顶点、退化面、法向翻转。
    - 检查重复三角形。
    - 检测 non-manifold（同一边关联面 > 2）。
  - `collapse_edge()`：
    - 迁移顶点、更新面、删除无效面。
    - 更新邻接与版本号。
  - `cleanup_neighbors()`：清理无效邻接。
- 流程要点：
  - 初始化：`build_adjacency()` + `compute_quadrics()` + 构建 heap。
  - lazy heap：每次 pop 校验版本；过期项丢弃。
  - 局部更新：只重算新点一环边并 push。
  - 终止：`F <= ceil(F0*ratio)` 或 heap 空。
  - 结束：`compact()` 后统计质量，并 `checkConsistency()`。
- 调用关系：
  - CLI/Viewer 都通过 `simplify_mesh()` 调用核心算法。

### src/core/Tsv.h / Tsv.cpp
- 对外函数：`append_tsv(path, model, ratio, stats, err)`
- 作用：
  - 若 TSV 不存在或为空，先写表头。
  - 每次简化写一行指标。
- 调用关系：
  - CLI 与 Viewer 在简化完成后调用。

### src/cli/main.cpp
- 作用：命令行工具 `mesh_simp`。
- 变量与参数：
  - 位置参数：`in.obj out.obj ratio`。
  - 可选参数：`--tsv path`、`--model_name name`、`--seed N`。
- 关键调用：
  - `Mesh::load_obj()` -> `simplify_mesh()` -> `Mesh::save_obj()` -> `append_tsv()`。
- 输出：标准输出仅打印简短摘要和 TSV 路径。

### src/viewer/Math.h
- 作用：基础矩阵运算。
- 函数：
  - `Mat4::identity()`
  - `Mat4::perspective()`
  - `Mat4::look_at()`
  - `operator*`（矩阵乘法）
- 调用关系：viewer 渲染矩阵计算。

### src/viewer/Shader.h / Shader.cpp
- 作用：GLSL shader 编译与链接。
- 函数：
  - `Shader::load(vs, fs)`：编译并链接。
  - `Shader::use()`：使用 program。
  - `Shader::destroy()`：释放资源。
- 调用关系：viewer 渲染管线使用。

### src/viewer/main.cpp
- 作用：OpenGL 可视化程序 `mesh_viewer`。
- 主要变量：
  - 相机控制：`yaw_deg`, `pitch_deg`, `distance_to_center`。
  - UI 状态：`ratio`, `csv_path`(TSV 路径), `model_name`。
  - Mesh/VBO/IBO：`input_mesh`, `output_mesh`, `gl_input`, `gl_output`。
- 关键函数：
  - `compute_center()` / `compute_radius()` / `reset_camera_for_mesh()`。
  - `build_render_data()`：从 Mesh 生成位置/法线缓冲。
  - `upload_mesh()` / `destroy_mesh()`：GL buffer 管理。
  - GLFW 回调：鼠标旋转/滚轮缩放。
- 核心流程：
  - 初始化 GLFW/GLAD/ImGui。
  - 加载 input OBJ。
  - UI 控制简化参数与执行。
  - 调用 `simplify_mesh()`，并 `append_tsv()`。
  - W 切换线框，B 切换前后，R 重置相机。

## 文件调用关系（简化版）

- CLI 调用链：
  - `src/cli/main.cpp` -> `Mesh::load_obj()` -> `simplify_mesh()` -> `Mesh::save_obj()` -> `append_tsv()`

- Viewer 调用链：
  - `src/viewer/main.cpp` -> `Mesh::load_obj()` -> `simplify_mesh()` -> `Mesh::save_obj()` -> `append_tsv()`

- 核心算法依赖：
  - `Simplify.cpp` 使用 `Mesh` 的邻接、quadric 与一致性检查。
  - `Mesh.cpp` 使用 `Vec3`/`Quadric`。

## TSV 输出指标与含义

字段顺序如下（制表符分隔）：

1) model
- 模型名称标签。

2) ratio
- 目标面数比例（F_out / F_in）。

3) V_in / F_in
- 输入顶点数、面数。

4) V_out / F_out
- 输出顶点数、面数。

5) time_total_ms
- 总耗时。

6) time_build_ms
- 初始化阶段耗时（邻接 + quadric + 初始 heap）。

7) time_simplify_ms
- 简化迭代耗时（heap pop + collapse + 局部更新）。

8) heap_push
- 堆插入次数（含初始与局部更新）。

9) heap_pop
- 堆弹出次数（含过期项）。

10) local_recompute_edges
- 局部重算边数量（体现 O(k log E)）。

11) skipped_invalid
- 非法 collapse 计数（翻转、退化、重复面等）。

12) nonmanifold_rejects
- 因生成 non-manifold edge 被拒绝的次数。

13) consistency_ok
- 简化后一致性检查是否通过（1/0）。

14) qem_err_mean
- 成功 collapse 的 QEM 代价均值。

15) qem_err_max
- 成功 collapse 的 QEM 代价最大值。

16) degenerate_faces
- 输出网格中面积过小的面数。

17) aspect_mean
- 三角形质量均值（max_edge^2 / (2*area)，越小越好）。
