# VoxelWorld

一个独立的 C++17 / Bullet3 星球碰撞实验。方案 1 按第三方插件的工作方式搭建基线。四种方案共享带撞击坑、坑缘和起伏的程序化星球，基准半径为 **25,600 米**。种子控制地形和查询回放。场值是沿星球径向测得的有符号高度差，形状查询按三线性场的零等值面计算。

| 方案 | 命令参数 | 渲染几何 | 地形碰撞 |
| --- | --- | --- | --- |
| 1 | `--scheme mesh` | SDF 采样后生成 Marching Cubes 三角形 | 以同一批三角形创建 Bullet 静态三角网格与 BVH |
| 2 | `--scheme grid` | 读取均匀体素块后生成 Marching Cubes 三角形 | 直接访问均匀体素块，可用 `--morton` 切换 4×4×4 块内 Morton 排布 |
| 3 | `--scheme dag` | 读取 HashDAG 后生成 Marching Cubes 三角形 | 直接遍历块内 HashDAG，精确去重相同值及八叉子树 |
| 4 | `--scheme vdb` | 读取 OpenVDB 后生成 Marching Cubes 三角形 | 通过 OpenVDB accessor 直接访问值。使用每块独立的 float 树，节点布局为 `Tree4<float, 3, 2, 2>` |

场查询与刚体接触共用三线性场。RayCast 按 DDA 遍历单元并求第一个零点。ShapeCast 对平移的球、胶囊和定向盒做时间与空间细分，记录细分后仍不确定的结果。字段刚体碰撞通过自定义 Bullet dispatcher 算法生成持久接触，多点约束、刚体间碰撞和求解交给 Bullet。模拟使用 60 Hz 固定步长和独立施加给每个刚体的径向重力。

## Windows 查看器

双击打包目录里的 `VoxelWorldViewer.exe`。查看器使用 Windows 原生 Direct3D 12，渲染实时生成的 Marching Cubes 网格，并用当前碰撞方案模拟刚体。默认同步两条移动 Invoker 路径。

| 操作 | 快捷键 |
| --- | --- |
| 切换 Mesh BVH、VoxelField、HashDAG、OpenVDB | `1`、`2`、`3`、`4` |
| 切换全星球和地表视角 | `F` |
| 绕视角中心旋转和缩放 | 左键拖动、滚轮 |
| 跟随下一个移动 Invoker | `Tab` |
| 开关 LOD 着色 | `L` |
| 向当前 Invoker 附近投放一组球、胶囊和盒子 | `B` |
| 暂停或重置模拟 | `Space`、`R` |

切换后端会从同一种子重建场景和刚体。状态栏显示 LOD 块数、三角形数量、刚体数量和 LOD 更新耗时。命令行可用 `--seed`、`--invokers`、`--scheme` 和 `--morton`。设备创建失败时会尝试 Direct3D 12 的 WARP 软件渲染，也可用 `--warp` 强制启用。受组织应用控制管理的电脑可能需要管理员批准本地生成的可执行文件。

### 在 Windows 上重新构建

安装 Visual Studio 2022 或 2026，选择「使用 C++ 的桌面开发」、Windows SDK 和 CMake。在项目根目录的 PowerShell 中运行构建脚本。首次构建会下载并校验 Bullet3、OpenVDB、oneTBB 和 Imath 源码，编译 Win64 Release 版本。添加 `-Test` 还会执行本机 CTest。 [Visual Studio C++ 安装说明](https://learn.microsoft.com/en-us/cpp/build/vscpp-step-0-installation)

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\build-windows.ps1
.\build-windows-native\dist\VoxelWorldViewer.exe
.\build-windows-native\dist\voxel_world.exe --scheme all --out results-windows
```

### 浏览器预览

双击生成的 `VoxelWorld.html` 可离线查看同一份 Marching Cubes 网格。支持全星球、地表、Invoker 位置切换和 LOD 着色。此预览展示导出时的地形快照，实时碰撞和动态块更新由原生查看器执行。预览把坐标量化到 1 米以减小文件体积。

```bash
./build/voxel_world --scheme mesh --global --frames 1 --warmup 0 --bodies 0 --obj --out results-windows-preview
python scripts/make-preview.py results-windows-preview/mesh.obj results-windows-preview/VoxelWorld.html
```

### Linux / WSL 的无窗口基准

已验证 Windows 下的 WSL Ubuntu 20.04。默认构建从带 SHA-256 校验的 Bullet3 **3.24** 源码包编译 CPU 库。使用系统 Bullet SDK 时可加 `-DVOXEL_SYSTEM_BULLET=ON`。 [Bullet3 源码](https://github.com/bulletphysics/bullet3) · [Debian 源码包](https://packages.debian.org/source/bookworm/bullet)

在 WSL 中进入 `/mnt/d/GitHub/VoxelWorld` 后执行。

```bash
sudo apt-get install build-essential cmake ninja-build libopenvdb-dev libtbb-dev libboost-dev libboost-iostreams-dev
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
(cd build && ctest --output-on-failure)
./build/voxel_world --scheme all --out results
```

下面的命令测试 `grid` 的 Morton 排布，并导出全星球渲染网格。OBJ `o` 名称含块坐标及 LOD，可用 Blender 等工具查看。基准计时排除 OBJ 写出时间。

```bash
./build/voxel_world --scheme grid --morton --out results-morton
./build/voxel_world --scheme mesh --global --frames 1 --warmup 0 --bodies 0 --obj --out results-planet
```

## Invoker、LOD 和测量

- `--invokers N` 控制移动的 Invoker 数量，默认两条不同路径。每个 1,600 米块按离它最近的 Invoker 选 LOD，样本间距为 50、100、200 或 400 米。相邻块共用边界采样，细块边界从粗块尺度插值。默认仅保留 Invoker 周围的地表块。`--global` 保留整个星球的地表块，远区使用粗 LOD。
- `--stream-every N` 指定多少帧更新一次块覆盖和 LOD，默认每六帧。已有块在局部 LOD 拓扑改变时重建，同步卸载旧碰撞体及其接触缓存。基准刻意分开记录流式更新和物理步进耗时。
- `--frames`、`--queries`、`--bodies`、`--seed` 和 `--warmup` 可调。默认每帧每类 12 次查询，48 个动态刚体，90 个测量帧。Headless 单线程运行四个独立物理世界，按同样的顺序重放输入。查询只包含地形。
- `summary.csv` 包含四类查询的 P50/P95/P99、命中、未判定、与网格方案的分歧及共同命中的 TOI 差值，并给出刚体步进、碰撞内存和初次构建统计。TOI 差值用扫掠路径上的米数表示，三角形与三线性零等值面之间本身存在几何差异。`frames.csv` 包含采样、存储构建、Marching Cubes、Bullet 注册和卸载、各 LOD 块数及内存估算。内存列是已发布数据的估算，单位为字节；分配器开销和构建临时峰值需要另行测量。

## 原型边界

此项目输出渲染网格文件，并通过独立 CLI 测量地形碰撞。混合 LOD 使用边界 SDF 插值，并把细网格边界折线贴到无歧义的粗网格轮廓上。交错符号的鞍形面仍可能有缝隙。碰撞覆盖以当前驻留的块为准，超出加载范围的查询结果只能解释为当前数据的结果。ShapeCast 固定朝向，时间和空间细分有预算，返回 `Unresolved` 时应把该次计为未判定。刚体使用离散接触，速度过高或特征小于采样间距时可能穿透。CCD、编辑挖掘和原生 UE 集成超出本次基准范围。

Marching Cubes 查表数据来自 VTK 9.3.1，许可见 [VTK-Copyright.txt](third_party/VTK-Copyright.txt)。
