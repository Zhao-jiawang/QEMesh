# QEM 网格简化项目说明报告

本文档给出算法原理、代码结构与每一步所用算法、复杂度分析，便于验收与后续报告撰写。

## 1. 原理说明（SIG’97 QEM Edge Collapse）

### 1.1 Quadric Error Metrics (QEM)
- 对每个三角面 (v0,v1,v2) 计算平面方程：
  - n = normalize((v1-v0) × (v2-v0))
  - d = -n · v0
  - p = [n.x, n.y, n.z, d]^T
  - K = p p^T
- 对每个顶点累加其 incident faces 的 K 得到顶点 quadric：
  - Q_v = Σ K
- 对一条边 (v1, v2)：
  - Q = Q_v1 + Q_v2
  - 代价函数：Δ(v) = v_hat^T Q v_hat, v_hat = [x, y, z, 1]^T
- 最优收缩点 v* 的求解：
  - 构造 A：
    [Q00 Q01 Q02 Q03]
    [Q10 Q11 Q12 Q13]
    [Q20 Q21 Q22 Q23]
    [ 0   0   0   1 ]
  - b = [0,0,0,1]^T
  - 解 A * v_hat = b 得到 v*；若不可逆或病态，回退到 {v1, v2, (v1+v2)/2} 中代价最小的点。

### 1.2 边收缩策略
- 仅对真实网格边建 pair（避免全点对）。
- 使用最小堆按代价选择最优收缩边。
- 收缩后仅更新“新点一环邻域边”的代价（局部更新）。
- 终止条件：F <= ceil(F0 * ratio) 或无合法 collapse。

### 1.3 鲁棒性约束
- 边界保持：
  - boundary-vertex 与 interior-vertex 禁止合并。
  - boundary-boundary 仅收缩到代价更小的端点。
- 防翻转/退化：
  - 若 collapse 后出现重复顶点、面积 < eps、或法向翻转（new·old < eps），判非法。
- 拓扑保护：
  - 禁止重复三角形（同顶点集合）。
  - 禁止 non-manifold edge（边关联面数 > 2）。
- 一致性检查：
  - 简化结束后检查邻接/面引用有效性。

## 2. 代码结构说明（含每一步算法）

### 2.1 数据结构层（src/core）

#### Vec3.h
- 算法：基础向量运算（加减、点积、叉积、归一化）。
- 作用：所有几何计算的基础。

#### Quadric.h
- 算法：平面 quadric 构造与误差评估。
- 作用：QEM 代价的核心数学结构。

#### Mesh.h / Mesh.cpp
- 结构：
  - `vertices` / `faces`
  - `vfaces`（顶点 -> 关联面集合）
  - `vneighbors`（顶点 -> 一环邻居）
- 步骤：
  1) OBJ 读取：支持 v/f，遇多边形用 fan 三角化。
  2) build_adjacency：构建 vfaces/vneighbors，统计边界。
  3) compute_quadrics：对每个面计算 K 并累加到顶点。
  4) compact：删除无效顶点/面并重建索引与邻接。
- 算法要点：
  - 邻接用紧凑集合存储，方便局部更新。
  - 边界检测使用“共享面次数”为 1 的边。

### 2.2 简化主流程（Simplify.h / Simplify.cpp）

#### 初始化阶段
- 算法步骤：
  1) `build_adjacency()`
  2) `compute_quadrics()`
  3) 遍历所有真实边 (v1,v2)，计算最优点与代价
  4) 将 edge pair 放入最小堆
- 复杂度：O(F) + O(E) + O(E log E)

#### 主循环（Edge Collapse）
- 算法步骤：
  1) 从堆中弹出最小代价边
  2) 校验版本号（Lazy heap）
  3) 进行几何/拓扑合法性检查
  4) 若合法，执行 collapse（更新顶点、面、邻接）
  5) 仅对新点一环邻域边重新计算代价并入堆
- 局部检查算法：
  - 重复面检测、面翻转检测、退化检测、non-manifold 检测。

#### 结束阶段
- 执行 `compact()` 清理无效元素。
- 统计输出指标：QEM 误差、退化面、aspect mean 等。
- 运行 `checkConsistency()`，记录是否通过。

### 2.3 CLI 与 Viewer

#### CLI (`src/cli/main.cpp`)
- 参数解析：in.obj out.obj ratio
- 可选参数：--tsv path、--model_name name、--seed N
- 调用链：
  - `Mesh::load_obj()` -> `simplify_mesh()` -> `Mesh::save_obj()` -> `append_tsv()`

#### Viewer (`src/viewer/main.cpp`)
- 使用 GLFW/GLAD/ImGui。
- 功能：加载模型、设置 ratio、简化并展示、线框切换、前后对比。
- 与 CLI 共用同一核心简化逻辑。

## 3. 复杂度分析

### 3.1 初始化复杂度
- 计算 quadric：O(F)
- 建立邻接：O(F)
- 建立所有边的 pair 并入堆：O(E log E)

### 3.2 每次 collapse 的复杂度
- 邻域更新：只更新新点的一环边，设一环邻居数为 k：
  - 重新计算代价：O(k)
  - 入堆：O(k log E)
- 不做全局重算，保证局部更新复杂度。

### 3.3 总体复杂度
- 约为：O(F + E log E + C * k log E)
  - C 为成功 collapse 次数
  - k 为平均一环邻居数

### 3.4 空间复杂度
- 顶点/面存储：O(V + F)
- 邻接：O(V + E)
- 堆：O(E)

## 4. 统计指标与用途（TSV）

- model：模型名
- ratio：目标面数比例
- V_in / F_in：输入顶点/面数
- V_out / F_out：输出顶点/面数
- time_total_ms：总耗时
- time_build_ms：初始化耗时
- time_simplify_ms：简化耗时
- heap_push / heap_pop：堆操作次数
- local_recompute_edges：局部重算边数量
- skipped_invalid：被判非法的 collapse 次数
- nonmanifold_rejects：因非流形边拒绝次数
- consistency_ok：一致性检查通过（1/0）
- qem_err_mean / qem_err_max：collapse 代价均值/最大值
- degenerate_faces：退化面数量
- aspect_mean：三角形质量指标均值
