# VoxelField 查询性能

## 多 Invoker 3D Clipmap

全星球回放使用 Ryzen 7 5800X、种子 42、48 个动态刚体、2 个移动演员 Invoker 和 1 个跟随演员移动的摄像机 Invoker。四个后端各跑 360 帧，每类查询 5,760 次，每两帧调度一次地形更新。四个后端使用相同的轨迹、查询输入和数据 LOD。原生查看器另在 Direct3D 12 硬件模式下生成了四种后端的 LOD 线框截图，Win64 和 WSL 的无窗口自检均通过。

### Win64 Release 查询

单次查询延迟以微秒计，格式为 P95 / P99。

| 方案 | RayCast | 球 ShapeCast | 胶囊 ShapeCast | 盒子 ShapeCast |
| --- | ---: | ---: | ---: | ---: |
| 网格 BVH | 2.60 / 4.30 | 25.90 / 31.10 | 37.20 / 46.20 | 48.40 / 58.20 |
| 均匀网格 | 1.30 / 2.00 | 3.60 / 5.90 | 3.20 / 4.90 | 3.70 / 9.10 |
| HashDAG | 2.00 / 2.90 | 4.30 / 6.20 | 3.80 / 5.20 | 4.40 / 9.10 |
| OpenVDB | 3.10 / 5.00 | 6.00 / 9.50 | 4.70 / 6.60 | 5.70 / 10.40 |

三种体素后端的命中状态、初始重叠标记和 TOI 距离逐条一致，TOI 比较容差为 10⁻⁸ 米。默认回放没有未判定查询。相对三角网格，RayCast 和胶囊各有 1 次命中分类差异，球和盒子完全一致。共同命中的 TOI 距离差 P95 为 0.38–0.49 米。Windows 上 OpenVDB 的 RayCast P95 和 P99 略高于网格 BVH，三类 ShapeCast 均更快。

### Win64 Release 移动成本与内存

179 次调度中有 29 次更新了块。各后端一共访问 10,496 个窗口进出位置，1,676 个块的有效 LOD 发生变化，连同受接缝影响的邻块共重建并替换 3,562 个块。初始加载单独统计。

| 方案 | 初始加载 ms | 完整更新 P95 ms | 调度 P95 ms | 碰撞发布 P95 ms | 无块变更 P95 ms | 地形峰值 MiB | 新旧并存估算 MiB | 进程峰值 MiB |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 网格 BVH | 791.06 | 322.90 | 0.634 | 41.511 | 0.0021 | 57.10 | 62.38 | 87.31 |
| 均匀网格 | 699.47 | 303.31 | 0.629 | 1.371 | 0.0024 | 46.75 | 61.80 | 75.96 |
| HashDAG | 1176.38 | 548.85 | 0.632 | 2.381 | 0.0023 | 69.30 | 93.47 | 113.59 |
| OpenVDB | 1171.62 | 426.54 | 0.677 | 8.400 | 0.0029 | 120.96 | 142.13 | 247.25 |

完整更新、调度和碰撞发布的 P95 按发生块变更的 29 次更新计算。完整更新包括调度、重采样、字段构建、Marching Cubes 和碰撞发布。原生查看器把字段和网格构建放在后台，发布前继续使用上一批地形。碰撞发布仍在主线程执行，查看器还会打包并上传 CPU 网格，这部分和 GPU 内存均未计入此表。

地形峰值包含字段、静态碰撞、CPU 渲染网格和调度元数据。新旧并存估算取当前地形加待发布的字段及网格的峰值。单后端独立进程的峰值取 Windows 的 `PeakWorkingSet64`，包含应用分配器和依赖库的开销。48 个刚体的固定步耗时 P95 依次为 1.18、0.99、1.11、1.41 ms，回放末尾各方案均无刚体中心低于解析地面 1 米以上。

### Win64 胶囊压力测试

全星球回放 120 帧，五类各 720 次。按网格 BVH、均匀网格、HashDAG 和 OpenVDB 的顺序，整体 P95 分别为 237.60、18.00、20.60、25.70 µs。擦边组的 P99 分别为 164.50、197.10、201.50、216.60 µs。三个体素后端结果一致，各有 2 次未判定，分别出现在擦边和跨块长扫掠。每个体素后端与 BVH 有 247 次命中分类差异，其中 244 次来自起始重叠组。

### WSL Release 交叉验证

同一条全星球轨迹在 WSL Ubuntu 20.04、GCC 9.4、Bullet3 3.24 和 OpenVDB 6.2 下的结果。进程 RSS 按单后端独立回放，用 `/usr/bin/time -f %M` 测量。

| 方案 | RayCast P95 µs | 球 P95 µs | 胶囊 P95 µs | 盒子 P95 µs | 完整更新 P95 ms | 地形峰值 MiB | 进程峰值 RSS MiB |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 网格 BVH | 3.42 | 22.94 | 31.54 | 46.65 | 367.96 | 60.80 | 92.14 |
| 均匀网格 | 1.62 | 3.26 | 2.76 | 3.27 | 336.83 | 50.56 | 81.63 |
| HashDAG | 2.76 | 4.19 | 3.38 | 4.13 | 737.91 | 73.11 | 117.25 |
| OpenVDB | 2.95 | 4.20 | 3.11 | 3.76 | 431.99 | 125.11 | 318.80 |

关闭全星球回退、只保留 Invoker 窗口的地表时，180 帧回放中，网格 BVH、均匀网格、HashDAG 和 OpenVDB 的地形峰值分别为 26.52、25.99、39.84 和 37.62 MiB。

WSL 的 3,600 次胶囊压力查询中，四个后端的 P95 依次为 189.96、15.55、17.38、17.78 µs。命中和未判定数与 Windows 一致。擦边组的均匀网格 P99 为 130.84 µs，BVH 为 126.72 µs。

### 复跑

```powershell
.\build-windows-native\dist\voxel_world.exe --self-test
.\build-windows-native\dist\voxel_world.exe --scheme all --global --frames 360 --queries 16 --stream-every 2 --out results-clipmap-win
.\build-windows-native\dist\voxel_world.exe --scheme all --global --capsule-stress --frames 120 --queries 30 --stream-every 2 --bodies 0 --out results-clipmap-win-capsule
.\build-windows-native\dist\VoxelWorldViewer.exe --capture-dir results-clipmap-native
```

```bash
./build/voxel_world --scheme all --global --frames 360 --queries 16 --stream-every 2 --out results-clipmap
./build/voxel_world --scheme all --global --capsule-stress --frames 120 --queries 30 --stream-every 2 --bodies 0 --out results-clipmap-capsule
./build/voxel_world --scheme all --frames 180 --queries 12 --stream-every 2 --out results-clipmap-local
for scheme in mesh grid dag vdb; do
  /usr/bin/time -f %M -o "results-clipmap/peak-$scheme.kib" ./build/voxel_world --scheme "$scheme" --global --frames 360 --queries 16 --stream-every 2 --out "results-clipmap/rss-$scheme"
done
```

`summary.csv` 给出查询延迟与准确性，`streaming.csv` 汇总移动和内存，`frames.csv` 记录逐帧的调度、网格构建、碰撞发布及各 LOD 块数。

## 旧版距离 LOD 基线

单线程 Release 实测环境为 Ryzen 7 5800X、WSL Ubuntu 20.04、GCC 9.4、Bullet3 3.24 和 OpenVDB 6.2。每组方案重放同一种子、相同的移动 Invoker 轨迹与查询。网格基线使用 Bullet 的静态三角网格 BVH，扫掠形状在栈上构造。计时包含宽相、字段访问、求交和未判定查询的全部开销。

### 默认回放

种子 42，2 个移动 Invoker、48 个刚体、90 帧，每种查询 1,080 次。表中单位为微秒，格式为 P95 / P99。

| 方案 | RayCast | 球 ShapeCast | 胶囊 ShapeCast | 盒子 ShapeCast |
| --- | ---: | ---: | ---: | ---: |
| 网格 BVH | 3.81 / 6.72 | 25.11 / 36.38 | 32.99 / 51.38 | 50.44 / 71.44 |
| 均匀网格 | 1.44 / 2.39 | 3.68 / 7.63 | 2.92 / 15.77 | 3.59 / 21.90 |
| HashDAG | 2.43 / 3.45 | 4.05 / 6.67 | 3.16 / 9.09 | 3.73 / 23.48 |
| OpenVDB | 1.82 / 3.01 | 3.97 / 6.87 | 3.13 / 5.60 | 3.67 / 13.67 |

种子 137 和 271828 的默认回放，以及种子 42 的全星球加载回放，同样在四种查询的 P95、P99 上低于网格 BVH。四组共测 25,440 次查询/方案，字段方案的命中分类与网格一致，未判定数为 0。种子 42 默认回放的共同命中 TOI 距离差 P95 在 0.37–0.42 米之间。

### 胶囊压力测试

随机变化胶囊半径、圆柱段长度和朝向，各类 540 次/种子。以下为种子 42 的 P95，单位为微秒。

| 场景 | 网格 BVH | 均匀网格 | HashDAG | OpenVDB |
| --- | ---: | ---: | ---: | ---: |
| 倾斜 | 76.67 | 6.02 | 8.00 | 8.37 |
| 擦边 | 112.95 | 42.83 | 45.50 | 45.05 |
| 跨块长扫掠 | 278.34 | 32.05 | 37.60 | 25.42 |
| 明确未命中 | 1.94 | 1.32 | 1.35 | 1.39 |
| 起始重叠 | 32.54 | 5.33 | 6.33 | 6.29 |

三个种子下，三种字段方案在五类场景各自的 P95、P99 均低于 BVH。擦边的差距最小，例如种子 42 的擦边 P99 为 102.81–114.78 µs，BVH 为 138.16 µs。三个种子合计 8,100 次压力查询/方案，各字段方案出现 4 次未判定。

与网格的 615 次命中分类差异/字段方案中，605 次出在专门的起始重叠组。字段能判断体积重叠并返回 `t=0`，向外扫掠时 Bullet 三角网格回调可能返回未命中。其余 10 次发生在压力组的其他场景，结果会受三线性零面和三角化表面的差异影响。

### 优化与代价

- RayCast 用块内行位掩码跳过同号单元，DDA 增量步进。连续段直接生成三次多项式，在单调区间里用有边界的 Newton 迭代和二分找首根。长斜向查询按扫掠体的时间区间枚举块，避免遍历整个三维包围盒。
- 均匀网格批量读取相邻行，HashDAG 的八角采样复用共同父路径，OpenVDB 的八角采样在同一叶节点内直接读取。查询仍访问各自的原始字段存储，公共索引只保留单元标记。
- ShapeCast 用字段支撑点的射线求出命中上界，用单元边上的零点截取零面的包围盒。根据形状的分离轴，以及三线性切平面的余项和单元边界限制，排除无法在上界之前接触的时间区间。对剩余区域继续细分，保留 0.75 米的搜索步长容差和显式 `Unresolved` 返回。
- 种子 42 的默认回放中，行位掩码使每个字段方案的峰值已发布碰撞内存增加约 0.23 MiB。总量为均匀网格 3.17、网格 BVH 4.22、HashDAG 6.94、OpenVDB 6.34 MiB。全星球加载时，当前每块独立 OpenVDB 树约为 80.65 MiB，均匀网格约为 11.18 MiB。查询优势与内存占用需要一起评估。

### 旧版命令

```bash
./build/voxel_world --scheme all --out results-query-opt
./build/voxel_world --scheme all --seed 137 --frames 120 --queries 20 --out results-query-opt-137
./build/voxel_world --scheme all --seed 271828 --frames 120 --queries 20 --out results-query-opt-271828
./build/voxel_world --scheme all --global --frames 24 --queries 20 --out results-query-opt-global
./build/voxel_world --scheme all --capsule-stress --frames 90 --queries 30 --bodies 0 --out results-capsule-opt
```

这些命令复现旧版参数。当前版本会使用 3D Clipmap。胶囊压力组可用 `--seed 137` 或 `--seed 271828` 切换种子。`summary.csv` 给出总延迟和同一输入的命中对照，`capsule-cases.csv` 按场景给出对照。
