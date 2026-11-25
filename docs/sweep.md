# 场后端形状 sweep（`fieldSweep`）原理

本文只讲 `--sweep` 模式里**场后端**（grid / dag / vdb）的形状扫掠如何工作，以及它和 mesh 后端（Bullet 三角面 CCD）为什么不可能给出同一个答案。

代码位置：`src/collision.cpp`（算法）、`src/main.cpp`（基准与报告）。
行号以当前版本为准，改动后需重新核对。

---

## 0. 一句话概括

不是"把形状拆成三角形逐点求交"，而是**保守的扫掠 AABB 递归**：把"形状扫过一条线段"化成"线段 ⊕ 形状（闵可夫斯基和）与三线性零等值面求交"，用一组**必要条件**（盒相交、SAT、场值下界、切平面）把候选区间成块否证掉，只对残余区间做采样与解析求根，并在预算/容差耗尽时**明确拒答**。

它保证"承诺的结论有证明"，不保证"一定给结论"。

---

## 1. 问题形式化

形状 `S`（凸：box / sphere / capsule）从 `from` 平移到 `to`，无旋转。记 `Δ = to − from`、`speed = |Δ|`、`f` 为分块三线性场（负值在地形内部）。

接触发生当且仅当：

```
∃ t ∈ [0,1], ∃ s ∈ S :  f(from + Δ·t + s) ≤ 0
⇔ ∃ t : min_{s∈S} f(from + Δ·t + s) ≤ 0
⇔ ∃ t : f_support(from + Δ·t) ≤ 0          // “按支撑函数膨胀后的场”
```

`f_support` 无法显式求值，所以整套算法是：**用上界（必要条件）逐块剪掉，再在残余块上做精确求解**。

输出是 `Hit{state, fraction, position, normal, initial}`：

- `fraction ∈ [0,1]` 即 TOI（沿路径的归一化参数），`fraction × |Δ|` = 沿路径距离；
- `initial = (fraction == 0)` 表示出发即重叠，与 TOI 语义无关，单独计数；
- `state ∈ {Contact, Miss, Unresolved}`，`Unresolved` 时 `fraction` 是**未证明区间的起点**，不是接触位置。

---

## 2. 外层三步

### 2.1 取一个已验证的 TOI 上界（`collision.cpp:725-736`）

```cpp
Hit fieldSweep(const Chunks& chunks, const Shape& shape, Vec from, Vec to) {
    SweepSearch search(shape, from, to);
    const Hit center = fieldRay(chunks, from, to);                  // 形状中心那条射线
    if (center.initial) return center;
    if (center.state == Hit::State::Contact) {
        const Vec offset = support(shape, center.normal * -1);       // 沿表面法线反方向的支撑点
        const Hit extreme = fieldRay(chunks, from + offset, to + offset);
        search.hit = extreme.fraction < center.fraction ? extreme : center;
        if (search.hit.initial) return search.hit;
    }
```

- 中心射线命中 → 至少存在一个接触，给出一份**上界 TOI**（真实答案只可能更早）。
- 再沿命中法线的反方向取形状支撑点（球是半径端点、capsule 是端盖、box 是角点）投第二条射线，取更早者。
- 有了上界，后续所有候选都能用 `cutoff()`（`collision.cpp:532-535`）剪枝：

```cpp
double cutoff() const {
    return hit.state == Hit::State::Contact && speed > 0
        ? std::max(0.0, hit.fraction - sweepToleranceCurrent / speed) : hit.fraction;
}
```

即"比已证接触晚 `tol/speed` 以上的区间一律不看"。

### 2.2 收集候选格子（`collision.cpp:737-759`）

| 层次 | 做法 | 作用 |
|---|---|---|
| chunk | `chunk.surfaceBounds`（chunk 内零面的包围盒）扩 `extents` 后做线段 clip | 不碰路径的 chunk 整体跳过 |
| cell（起点） | `forIndexedCells(chunk, Box{from,from}.expanded(extents), true, ...)` | 起点所在/覆盖的格子先处理（通常是初始重叠的来源） |
| cell（扫过） | `forSweptCells`（`collision.cpp:142-181`）沿三轴 DDA 推进，给出每格被扫过的 `[enter, exit]` | 沿路径逐格推进，代价 ≈ 沿路径格子数 |
| 候选盒 | **`zeroBounds(cell)`，不是整格**（`collision.cpp:334-360`） | 用 8 个角值 + 边上线性插值求出该格内零面的局部包围盒；cell 可能 50 m，零面也许只占几米 |

候选按 `first` 排序后依次 `visit()`（`collision.cpp:744-750`），早接触能尽快抬高 `search.hit`，把后面的候选整体剪掉。

### 2.3 逐候选细化

`search.visit(cell, Trilinear(cell), box, first, last, seed=true)`，见下节。

---

## 3. `visit()`：六级保守剪枝（`collision.cpp:569-676`）

每一级都只有两种结局："**能证明无接触 → 返回**"或"不能证明 → 继续"。全部通过才承认"这里可能存在接触"，并把时间区间收缩到 `[first, last]`。

| 级 | 判据 | 依据 / 为什么保守 |
|---|---|---|
| 0 | `first > last` 或 `hit.initial` | 区间被前序剪枝清空 / 已命中可提前退出 |
| 1 | `clip(box.expanded(extents), from, delta, first, last)` | 路径 AABB 与 `box ⊕ extents` 不相交。`extents` 是形状在**任意朝向下**沿各轴的最大半宽，是安全外包络 |
| 2 | 用路径扫过段再裁 `box`（`collision.cpp:580-586`） | 把盒子缩到路径真正经过的那一段，让后续 `half` 变小 |
| 3 | **12 条 SAT 轴**（`clipAxis`，`collision.cpp:436-446`） | box 轴(=cell AABB 的三条世界轴) × 形状轴的叉积。box 形状需 3(形状轴)+9(叉积)=12；sphere 各向同性 → 0 条；capsule 1+3=4 条。解析地给出可行时间区间 |
| 4 | `clipRounded`（`collision.cpp:447-462`） | SAT 对**带圆角**形状与盒子不精确（圆角会突破 SAT 的角点包络）：取一个姿态求实际 closest point，间隙非零就用该方向当新轴再 clip 一次 |
| 5 | **场值下界**：`centerValue − |g|·half + residual > ε ⇒ 整盒无零面` | trilinear 的二次/三次项不能只靠切平面界定；`Trilinear::lower()`（`collision.cpp:205-211`）用 `xy/xz/yz/xyz` 的四个 ± 组合 + 三次项绝对值，给出**严格下界** |
| 6 | 切平面 + 面约束（`clipPlane` / `clipConstrainedPlane`，`collision.cpp:463-530`） | 接触必要条件 `g·pose − supportRadius(g) ≤ threshold`，`threshold = g·center − f(center) − residual + ε`。线性化只在 cell 内有效；支撑点跑到 cell 外时，用盒面不等式把线性项补齐（最多叠两轮） |

> 注意：第 5、6 级都依赖**三线性场**，所以整套算法的"真值"是 trilinear 零点集，而不是 MC 三角面。这是后面 §8 分歧的根源。

---

## 4. 采样与解析根

通过剪枝后，代码并不二分区间，而是：

1. **采样关键姿态**（`sample()`，`collision.cpp:558-567`）：把点投影到形状表面（`projectToShape`），取该点场值；`> 2ε` 说明不在内部，丢弃；否则记 `Hit`。只接受**更早**的时间（`time > hit.fraction` 直接返回），所以是单调改进。
2. 采样点：`first` 处的极端点/最深点/中心、`midpoint` 同样三点、`last` 处最深点。圆心/最深点覆盖"最先接触"的典型位置。
3. **`seed` 分支做解析求根**（`collision.cpp:610-623`）：沿支撑点投一条射线，把三线性场代入参数方程得到**三次多项式**，`firstRoot()`（`collision.cpp:214-250`）解析求根（含三次/二次退化与根的分段处理）。因此在 cell 内部它给出的是**精确零点**，不是"采样到的最接近值"。

---

## 5. 分割与收敛（`collision.cpp:626-676`）

```cpp
const double travel = speed * (last - first);
if (speed * priorSpan > 2*tol && last - first < priorSpan * 0.4) { visit(同区间); return; } // 防抖
if (travel <= tol && dim[axis] <= tol) { ...收敛/丢弃/细分/放弃... }
if (travel > tol && travel >= dim[axis]) { 时间二分：[first,mid] + [mid,last]; }
else { 空间二分：沿最大轴把 box 切成两半; }
```

- **防抖**：区间没实质缩小（<40%）而相对精度仍超预算时，原地再试一轮，避免"切了但没变小"的无意义分割。
- **时间主导**（路径长、盒子小）→ **时间二分**：保守推进，把一次大位移拆成多次小位移。
- **空间主导**（盒子大、路径短）→ **空间二分**：沿最大轴切，`half` 减半 → §3 第 5 级下界收紧、第 6 级切平面更贴近真值。

两种分割都满足两条性质：**子区间并集 = 父区间**、**子盒并集 = 父盒**，且父区间只有在前几级**证明无接触**时才整体丢弃 —— 这就是"不漏"的来源（在"场=trilinear 零面"这个前提下）。

---

## 6. 三个拒答出口（诚实性机制）

| 出口 | 位置 | 条件 | 记账 |
|---|---|---|---|
| 预算耗尽 | `collision.cpp:597-605` | `budget-- <= 0`（默认 `sweepBudget=48000` visit/cast） | `markUnknown(first)`、`budgetStops++` |
| 容差地板 | `collision.cpp:638-658` | `travel ≤ tol 且 cell ≤ tol` | 见下 |
| 最终裁决 | `collision.cpp:772-777` | 已证接触 + 未证明区间起点早出预算 | 返回 `Unresolved(unknown)` |

容差地板的三种处理（`collision.cpp:638-658`）：

```cpp
const bool harmless = hit.state == Hit::State::Contact &&
    (hit.fraction - first) * speed <= 2 * tol;     // 这条带藏不住 >2*tol 的提前量
if (!harmless && refinements < 32) { ++refinements; /* 空间二分后再来 */ }
markUnknown(first); ++toleranceStops;              // 仍说不清 → 记账
```

- **无害带直接丢弃**：宁可少记一个 `unknown`，因为它的物理含义是"最多早 2*tol"，在预算内；
- **有害带继续细分**（每 cast 上限 `sweepRefinementLimit = 32`）：用几何换诚实，而不是放宽尺子；
- 只有细分也无解时才记 `unknown`。

最终裁决**必须用区间起点**（`collision.cpp:772-777`）：

```cpp
const bool hidden = search.hit.state == Hit::State::Miss ||
    (search.hit.fraction - search.unknown) * search.speed > 2 * sweepToleranceCurrent;
```

因为隐藏接触可能出现在区间**起点**，起点才是"最多能提前多少"的保守估计；用终点会把提前量少算最多一个 `tol`（等价于把预算从 `2*tol` 悄悄放成 `≈3*tol`）。

---

## 7. 复杂度与性能特征

- 一次 cast 通常 **~300 次 visit**（`--cast-at` 可查 `visits/depth`）；
- 每次 visit 只读该 cell 的 **8 个角值**（32 B 紧邻，对 L1 友好），12 条轴的定长循环，**无堆分配**（`candidates` 每 cast 复用），`Trilinear` 只是 8 个 double 的闭包；
- 剪枝发生在**盒子层**：绝大多数候选在 1~4 级即整块否证，真正走到采样/求根的是极少数；
- 对比 mesh 侧：Bullet `convexSweepTest` 走 BVH + 每三角形一次 GJK/CCD；`btBvhTriangleMeshShape` 构造本身占 build 的 ~7.5%。

实测（75 invoker × 1000 帧 × 21 = 1,575,000 sweeps/后端，tol 0.75）：

| 后端 | mean | ratio |
|---|---|---|
| mesh | 34.9 ~ 36.4 µs | 1.0 |
| vdb | 5.50 ~ 6.02 µs | **0.154 ~ 0.167x**（场侧快约 6 倍）|

---

## 8. 为什么和 mesh 不可能一致（穿模的根因）

- 场后端的"真值"是 **trilinear 零等值面**（§3 第 5/6 级、§4 全部基于它）；
- mesh 后端与**屏幕上渲染的地形**是同一份场的 **MC 三角面**（`terrain.cpp:292 meshChunk`，四后端共用同一份 MC）；
- 表征尺度：`chunkSize 1600 / fineCells 32 = 50 m` per cell。形状半径远小于 cell 时（球 10 m、capsule 9.8/13 m），两张面在 cell 内可以差十几米。

实测（默认种子、tol 0.75、与 mesh 逐 cast 对比）：

| 方向 | 条数 | 占比 | 后果 |
|---|---|---|---|
| field **晚报**（`vdb_toi > mesh_toi`） | 1,347,195 | 85.5% | 已穿进可视地表才判接触 → **穿模**，深度 = delta |
| field **早报** | 225,444 | 14.3% | 提前停 → 悬空，无穿模但有可见间隙 |
| 完全一致 | 2,361 | 0.15% | — |

误差分布：mean 0.271 m、p95 1.060 m、max 24.933 m；`> 1.5 m (=2*tol)` 49,853 条（3.17%），`> 5 m` 7,644 条，`> 20 m` 13 条。

极值约等于**半个 cell**（25 m），说明这是分辨率上界而非算法缺陷：`fineCells` 翻倍 → 上界减半。

工程上的三层防护（代价递增）：

1. **保守前移**：TOI 里扣掉安全余量，把"穿模"换成"悬空 tol"；
2. **接触后校正**：在 TOI 位姿取场值，若为负沿梯度推出（场后端唯一可行的 depenetration）；
3. **局部加密**：`fineCells` 可调 / 玩家附近 chunk 用更高分辨率。

---

## 9. CLI 与输出

```
.\voxel_world.exe --sweep --invokers 75 --frames 1000 --shapes-per-invoker 21 \
                 --compare vdb --sweep-budget 48000 --sweep-tolerance 0.75 \
                 --sweep-csv sweepMesh2vdb.csv
```

相关开关：

| 开关 | 含义 |
|---|---|
| `--sweep-budget N` | 每 cast 的 visit 预算（默认 48000，实测恒 0 触顶） |
| `--sweep-tolerance METERS` | 容差地板与裁决尺度（0.01..10，默认 0.75） |
| `--mismatch-samples N` | 打印多少条 mismatch（0 = 全打） |
| `--cast-at FRAME,INDEX` | 单条 cast 定点调试：visits/depth/stops、已证接触与 mesh 的 gap、最早/最晚放弃区间 |
| `--sweep-csv PATH` | 逐 cast 结果 CSV（来自**不计时**的校验遍） |

CSV 列：

```
frame,index,kind,span_m,mesh_state,mesh_toi_m,<backend>_state,<backend>_toi_m,delta_m
```

`delta_m` 只在两边都是 `contact` 时填充（避免把 `Unresolved` 的 band 起点当接触位置相减）。

报告行含义见 §10。

---

## 10. 报告行速查

```
vdb vs mesh [sweep]: 75 static invokers x 21 shapes x 1000 frames = 1575000 sweeps per backend (identical list)
  build mesh 21462.937 ms, vdb 26402.763 ms          # 建世界，不计入查询性能
  mesh: total 54933.750 ms, mean 34.879 us, 28670.899 sweeps/s
  vdb: total 8666.242 ms, mean 5.502 us, 181739.677 sweeps/s
  ratio vdb/mesh: 0.158x                             # <1 表示场更快
  contacts mesh ..., vdb ..., initial 0, conflicts 0, refusals 0
  states mesh A/B/C, vdb A/B/C (contact/miss/unresolved)
  sweep stops: budget N (limit X), tolerance M (limit T m)
  paired N, p95 TOI difference .., max .. m
  mismatch: frame F, shape I (kind), ...
```

- `conflicts` = 两后端状态不同且都不含 `Unresolved`（contact vs miss 对立）；
- `refusals` = 有一方 `Unresolved`；
- `paired` = 两边**都**报 `contact` 的条数 —— 只说明状态相同，**不说明位置一致**（所以 `conflicts 0` 旁边仍可能有 24.9 m 的 `max`）。

基准的输入约定（`main.cpp:823-893`）：invoker 位置**全程静止**，世界只 `update()` 一次，帧循环里只有 sweep；每帧用 `seed ^ (salt*(frame+1))` 重新掷一批 cast，`from = surface + dir*(support+120)`、`to = surface − dir*(support+60)`，保证恰好穿过表面一次。因此这是**纯查询吞吐**基准，不含流式/更新成本。

---

## 11. 已知边界与后续方向

- `Unresolved` 在 tol 0.75 上残留 0~2 条、tol 0.3 上 4~10 条，均为**几何层真分歧**（capsule 因半径小于 cell，`--cast-at` 可见"字段已证接触比 mesh 早 16.7 m"）；
- 缺 `--sweep-strict METERS`（把 `2*tol` 换成显式绝对值，用于刀刃 case 的可读承诺）；
- 缺 `p50/p99`（逐帧内层计时后无逐条时间戳）与"超预算条数"内建计数（目前靠 CSV 离线统计）；
- `fineCells` 不可调（根治穿模上界的唯一结构性手段）。
