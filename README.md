# VoxelWorld

一个独立的 C++17 / Bullet3 星球碰撞实验。方案 1 按第三方插件的工作方式搭建基线。四种方案共享带撞击坑、坑缘和起伏的程序化星球，基准半径为 **6,371,000 米（地球尺度）**。种子控制地形和查询回放。场值是沿星球径向测得的有符号高度差，形状查询按三线性场的零等值面计算。

在 Windows 上运行 `scripts\build-windows.ps1` 构建原生 DX12 查看器，再打开 `build-windows-native\dist\VoxelWorldViewer.exe`。按 `F` 切换地表视角，按 `K` 显示各 Invoker 的数据范围。完整命令见[在 Windows 上重新构建](#在-windows-上重新构建)。

## 大世界坐标与大气

CPU 的 `Vec`、相机、网格顶点、变换、LOD 和碰撞查询均使用 double；Bullet 源码也以 `BT_USE_DOUBLE_PRECISION` 编译。使用 `VOXEL_SYSTEM_BULLET=ON` 时，必须提供以同一宏编译的双精度 SDK。体素存储中的 float 是局部有符号距离值，不承载世界坐标。

原生 GPU 上传边界将 double 拆成 `hi=float(value)`、`lo=float(value-double(hi))`。顶点着色器用 df64 先减去摄像机高低位，再转为 float 做相机相对空间投影，参考 [UE Large World Coordinates 的 DoubleFloat 和 translated world space](https://dev.epicgames.com/documentation/en-us/unreal-engine/large-world-coordinates-rendering-in-unreal-engine-5)。地形、刚体、标记和窗口线框共用此链路。深度使用 reverse Z，避免地球尺度远平面压缩近处深度精度。

`src/shaders/df64.hlsli` 实现 Andrew Thall《Extended-Precision Floating-Point Numbers for GPU Computation》的 Two-Sum、Fast-Two-Sum、4097 Split、Two-Product，以及 df64 加减乘除、校正平方根。所有误差变换使用 HLSL `precise`，编译启用 IEEE strictness，避免乘加融合或重排破坏误差项。采用论文的双 float 分支（约 48 位有效数字、float 指数范围），没有实现四 float 的完整 qf128 数学库，也不依赖硬件 FP64。

`src/shaders/atmosphere.hlsl` 用 df64 求大气射线区间和起点高度，然后沿局部射线求半径平方差，由 `(r² - R²) / (r + R)` 求各积分点的高度。大气默认厚度为 100 公里，采用 Rayleigh/Mie 单次散射、太阳遮挡、视线消光和地形空气透视。每条视线使用 16 个积分样本，每个样本使用 6 个太阳光路径样本。相机与星心先在 CPU double 中相减再拆分，支持带巨大公共平移的行星场景。 [大气散射的相函数和积分关系](https://ebruneton.github.io/precomputed_atmospheric_scattering/atmosphere/functions.glsl)

体素场使用八叉树叶节点覆盖整个星球的包围立方体。每个方向的块数根据星球半径向上取整为 2 的幂，当前边界为 ±6,553,600 米。每个 Invoker 和摄像机都细分同一棵树，重叠区域保留最细层。默认 LOD0 采样间距为 50 米，可用界面或命令行切换。采样间距随 LOD 逐级翻倍，LOD4 起块的边长也逐级翻倍。当前表面最粗层间距为 204.8 公里，完整覆盖其余星球。实体内部和空区使用常量存储叶节点，避免将整个体积展开成密集体素。

相邻叶节点在面、边、角保持 2:1 采样间距平衡，边界场值从粗层一致插值，细网格边界贴合对应粗 Marching Cubes 单元的轮廓。四种后端都从这些叶节点生成网格，粗层同样参与地形查询与碰撞。原生查看器默认启用此全局分层覆盖；CLI 使用 `--global` 启用，未指定时保留局部窗口基准模式。`--global --obj` 导出的全部三角形均来自体素场，不再附加远景球网格。天空着色器只负责大气，不再绘制或写入替代地面的解析球面；线框模式先写实体网格深度，避免背面线条穿透。地形起伏、撞击坑和 Invoker 移动仍保持原来的米级空间尺度。

Release 验证命令如下。`voxel_gpu_precision` 默认在 WARP 上运行相同的发布版 HLSL，也可用 `--hardware` 验证实际显卡。

```powershell
ctest --test-dir build-windows-native -C Release --output-on-failure
.\build-windows-native\Release\voxel_gpu_precision.exe --hardware
.\build-windows-native\Release\VoxelWorldViewer.exe --capture-dir results-earth-captures
.\build-windows-native\Release\VoxelWorldViewer.exe --warp --capture-dir results-earth-warp
```

GPU 测试包含 94 组数据，检查 df64 运算、毫米高度、地平线求交、轨道距离、10⁹/10¹² 米绝对坐标的相对消减，以及大气输出有限性和透射率范围。CPU 测试包含正负百万米坐标的射线、形状扫掠和刚体静止接触。截图验证覆盖四种后端的轨道、地表、高空地平线、线框与 LOD 窗口，另生成公共平移 `(10¹², -10¹², 10¹²)` 米前后的两张对照图。用 `python scripts/validate-render-captures.py results-earth-captures` 检查对照图。另有全局体积覆盖、32 个方向的远处地形查询、2:1 平衡、粗细块字段连续性，以及摄像机跨半球时的后台替换验证。单次散射与固定样本数是实时近似，远处地形按粗层采样，不保留细小起伏；大气没有多次散射和云层。

| 方案 | 命令参数 | 渲染几何 | 地形碰撞 |
| --- | --- | --- | --- |
| 1 | `--scheme mesh` | SDF 采样后生成 Marching Cubes 三角形 | 以同一批三角形创建 Bullet 静态三角网格与 BVH |
| 2 | `--scheme grid` | 读取均匀体素块后生成 Marching Cubes 三角形 | 直接访问均匀体素块，可用 `--morton` 切换 4×4×4 块内 Morton 排布 |
| 3 | `--scheme dag` | 读取 HashDAG 后生成 Marching Cubes 三角形 | 直接遍历块内 HashDAG，精确去重相同值及八叉子树 |
| 4 | `--scheme vdb` | 读取 OpenVDB 后生成 Marching Cubes 三角形 | 通过 OpenVDB accessor 直接访问值。使用每块独立的 float 树，节点布局为 `Tree4<float, 3, 2, 2>` |

场查询与刚体接触共用三线性场。RayCast 按 DDA 遍历单元并求第一个零点。ShapeCast 对平移的球、胶囊和定向盒做时间与空间细分，记录细分后仍不确定的结果。字段刚体碰撞通过自定义 Bullet dispatcher 算法生成持久接触，多点约束、刚体间碰撞和求解交给 Bullet。模拟使用 60 Hz 固定步长和独立施加给每个刚体的径向重力。

字段块在生成时建立零面和实心单元的行位掩码。查询用位掩码与扫掠区间跳过空单元，并在候选单元批量读取实际存储的八个角值。ShapeCast 用支撑点射线提供已确认的命中上界，再通过零等值面包围盒、分离轴和三线性切平面余项排除更早的接触，难例继续细分。行位掩码计入字段内存，构建时间计入生成网格的阶段。

## Windows 查看器

### 延迟着色、HDR 与 CSM

DX12 地形顶点按空间分组缓存到 GPU。主视图和每级阴影各自剔除落在绘制范围外的分组。

原生查看器的 RGBA8 G-buffer 保存地形高度与坡度权重、其他物体的线性材质颜色及材质标记。RGBA16F G-buffer 保存世界法线与粗糙度，另有可读取的 D32 反向深度。全屏光照阶段从深度重建相机相对位置，计算方向光、环境光、镜面高光、级联阴影和大气空气透视。天空和调试线框也在线性 HDR 空间合成，最终统一处理曝光、ACES filmic 近似曲线和 sRGB 输出转换。场景颜色使用 `R16G16B16A16_FLOAT`，可以保留大于 1 的亮度；当前显示输出为色调映射后的 sRGB/SDR。[HDR 渲染参考](https://github.com/microsoft/DirectXTK12/wiki/Using-HDR-rendering) · [色调映射曲线来源](https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/)

地形着色使用程序化海拔与其梯度生成的平滑法线。随着网格采样间距和每像素覆盖面积增大，细节逐步过渡到星球径向法线和基准颜色。远景大气光程也平滑过渡到参考球面深度。截图验证在关闭大气和阴影的轨道画面中检查环纹及块边界。

CSM 使用四张深度图，默认每张 1024×1024，按对数与均匀混合方式划分视锥。CPU 使用 double 拟合包围球并在星球坐标系内对齐阴影纹素，GPU 继续在相机相对坐标系计算阴影。接收面使用法线偏移，3×3 PCF 过滤阴影边界，级联末尾 10% 混合到下一层。默认阴影距离随相机高度在 10–200 公里之间调整，末级向无阴影光照渐变，轨道视角超出局部阴影距离时关闭局部 CSM。[CSM 参考](https://learn.microsoft.com/en-us/windows/win32/dxtecharts/cascaded-shadow-maps)

`Shadows [J]` 或 J 切换阴影，`[` / `]` 每次降低／提高 0.5 EV。命令行 `--exposure EV` 接受 -10 到 10，默认 0；`--no-shadows` 关闭 CSM。例：

```powershell
.\build-windows-native\dist\VoxelWorldViewer.exe --exposure -0.5
.\build-windows-native\dist\VoxelWorldViewer.exe --no-shadows
```

`--capture-dir` 在可用时启用 D3D12 调试层，并采集阴影开关、三档分辨率、曝光 +1 EV、色调映射、大气调整与窗口缩放的固定场景。`.pipeline.txt` 记录 G-buffer 有效像素数、HDR 浮点最大值、阴影贴图实际分辨率和四个级联的深度像素数。`python scripts/validate-render-captures.py DIRECTORY` 检查画面对照和缓冲报告。GPU 数值测试还对照 CPU 检查曝光与色调映射结果。

### 渲染设置

工具栏 **`Render settings...`** 打开设置窗口。点击 `Apply` 即可应用到当前场景，`Defaults` 恢复初始参数。`J`、`H` 和 `[` / `]` 继续快速切换阴影、大气和曝光。

| 分组 | 可调参数 |
| --- | --- |
| CSM | 开关、每级 Shadow Map 分辨率（1024² / 2048² / 4096²）、距离上限、对数分割权重、法线和深度偏移倍率 |
| 大气 | 开关、厚度、Rayleigh / Mie 各自的尺度高度与散射强度、Mie 各向异性 |
| HDR | 曝光（−10 到 +10 EV）和色调映射（ACES、Reinhard、线性裁剪） |

四张 D32 阴影贴图的显存用量分别约为 16、64、256 MiB。切换分辨率时会重建阴影资源。HDR 场景仍通过色调映射输出到 sRGB/SDR 交换链。

命令行可用 `--shadow-resolution`、`--shadow-distance-km`、`--shadow-split-blend`、`--shadow-normal-bias`、`--shadow-depth-bias` 设置 CSM。大气对应 `--atmosphere-height-km`、`--rayleigh-height-km`、`--mie-height-km`、`--rayleigh-strength`、`--mie-strength`、`--mie-anisotropy` 和 `--no-atmosphere`。HDR 使用 `--exposure` 和 `--tone-map aces|reinhard|linear`。

```powershell
.\build-windows-native\dist\VoxelWorldViewer.exe --shadow-resolution 2048 --shadow-distance-km 80 --atmosphere-height-km 80 --exposure -0.5 --tone-map reinhard
```

双击打包目录里的 `VoxelWorldViewer.exe`。查看器使用 Windows 原生 Direct3D 12，渲染实时生成的 Marching Cubes 网格，并用当前碰撞方案模拟刚体。摄像机实际位置始终参与数据 LOD，默认还有两条移动 Invoker 路径。暂停时仍可移动摄像机并更新驻留范围。轨道相机远离地表时仍保留整个星球的粗层数据，靠近新的地表位置后会逐层细分。

| 操作 | 快捷键 |
| --- | --- |
| 切换 Mesh BVH、VoxelField、HashDAG、OpenVDB | `1`、`2`、`3`、`4` |
| 切换全星球和地表视角 | `F` |
| 进入／退出自由摄像机（从当前视角接管） | `C` 或 `Free [C]` |
| 自由摄像机前后左右移动 | `W` / `S` / `A` / `D` |
| 自由摄像机下降／上升 | `Q` / `E` |
| 自由摄像机转向、加速、调速 | 左键或右键拖动、按住 `Shift`（5 倍）、滚轮 |
| 绕视角中心旋转和缩放 | 左键拖动、滚轮 |
| 设置移动 Invoker 数量（1–16） | 工具栏 `Invokers` |
| 跟随并接管指定 Invoker | 工具栏 `Camera` 选择 `Invoker N`，或按 `Tab` 切到下一个 |
| 接管时沿地表移动、加速 | `W` / `A` / `S` / `D`，`Shift`（3 倍） |
| 脱离当前 Invoker | `Esc` 或 `Camera` 选择 `Detached`，也可用 `C` 接入自由相机 |
| 开关 LOD 着色 | `L` |
| 开关各 Invoker 的 3D LOD 窗口线框 | `K` |
| 设置 LOD0 半径并预览各级距离 | 工具栏 `LOD ranges...` |
| 切换地形和刚体的三角形线框渲染 | `F2` 或 `Wireframe [F2]` |
| 开关大气散射和地形空气透视（默认开启） | `H` 或 `Atmosphere [H]` |
| 开关 CSM 阴影（默认开启） | `J` 或 `Shadows [J]` |
| 调整 CSM、大气和 HDR | 工具栏 `Render settings...` |
| 降低／提高曝光 0.5 EV | `[` / `]` |
| 向当前 Invoker 附近投放一组球、胶囊和盒子 | `B` |
| 暂停或重置模拟 | `Space`、`R` |

自由摄像机以 double 保存位置，W/S 沿视线前后移动，A/D 左右平移，Q/E 沿相机上方向升降。初始速度随进入时的高度调整，状态栏显示基础速度；滚轮向上加速、向下减速，Shift 临时加速 5 倍。松开移动键或切换到其他应用时停止移动，暂停模拟仍可飞行并更新地形 LOD。该模式不做地形碰撞约束。再次按 C 返回先前的轨道／地表模式，F 切换至锚定视角，Reset 退出自由模式。工具栏会随窗口宽度自动换行。

`Invokers` 的数量仅指移动 Invoker，摄像机另外参与数据 LOD。调整数量保留现有刚体和剩余 Invoker 的位置。接管时使用第三人称相机，WASD 按镜头方向沿程序化地表移动，默认速度为 240 米/秒。拖动旋转镜头，滚轮调整跟随距离。换目标或脱离后，原 Invoker 从当前位置平滑回到当前巡航轨迹。暂停时仍可手动移动，巡航和回归在恢复模拟后继续。Invoker 标记使用分面法线、高光和边缘补光，当前相机目标为青色，其他标记为橙色。

切换后端会从同一种子重建场景和刚体。状态栏显示 LOD 块数、三角形数量、刚体数量和 LOD 更新耗时，并每 0.25 秒刷新内存统计（MiB）：`RAM` 为当前进程工作集，`Private commit` 为进程私有提交量，`Peak RAM` 为进程启动以来的峰值工作集。另列出当前驻留地形的估算总量及体素字段、CPU 网格、碰撞、调度元数据分项；该估算属于进程内存的一部分，不包含后台构建、动态刚体、渲染资源和分配器额外开销，不应与进程统计相加。命令行可用 `--seed`、`--invokers`、`--scheme` 和 `--morton`。设备创建失败时会尝试 Direct3D 12 的 WARP 软件渲染，也可用 `--warp` 强制启用。受组织应用控制管理的电脑可能需要管理员批准本地生成的可执行文件。

### 体素网格精度

工具栏 `Voxel step` 下拉菜单提供五个采样间距倍率，默认 `1`。倍率越小，Invoker 附近的网格越密；界面同时显示 LOD 0 的实际米数。

| 设置 | LOD0 采样间距 | 逻辑网格每边单元数 |
| --- | --- | --- |
| `0.125` | 6.25 米 | 2²¹ |
| `0.25` | 12.5 米 | 2²⁰ |
| `0.5` | 25 米 | 2¹⁹ |
| `1`（默认） | 50 米 | 2¹⁸ |
| `2` | 100 米 | 2¹⁷ |

切换后在后台重建体素场、网格和碰撞，完成后统一替换；摄像机、刚体位置及速度继续保留。状态栏 `Step` 显示当前生效的倍率，重建期间另显示目标倍率。全星球覆盖范围保持不变，最粗层随字段边界选取；近处加密会增加相应的 LOD 层级。切换后端和 Reset 会保留所选倍率。

原生查看器和命令行基准都支持 `--voxel-density`（也接受 `--density`），仅接受表中的五个值。

```powershell
.\build-windows-native\dist\VoxelWorldViewer.exe --voxel-density 0.125
.\build-windows-native\dist\voxel_world.exe --scheme grid --global --voxel-density 0.5 --frames 1 --warmup 0 --bodies 0 --obj --out results-density
```

CLI 的 `settings.txt` 保存倍率和 LOD 0 实际间距，OBJ 元数据及浏览器预览的图例也保留此设置。

### LOD 范围

点击工具栏的 `LOD ranges...`，填写 LOD0 的半径。单位为米，默认 512，只接受小于体素场边长的正整数 2 的幂。后续级别的半径自动逐级翻倍，直至第一次达到或超过体素场的边长，该级用于全场回退。当前场边长为 13,107,200 米，默认生成 LOD0–15。界面同时显示 `Voxel step` 对应的逻辑立方网格每边单元数，常量区域会保留粗节点。点击 `Apply` 后，后台更新受影响的体素、网格和碰撞。`K` 显示含滞回余量的已发布窗口，`L` 显示实际地表 LOD。增大精细层的范围会增加构建时间和内存。

全局 LOD 各层有独立锚点，步长同时取决于各级半径和体素间距。在 `0.125×` 与默认 512 米半径下，前四层分别按 25、50、100、200 米滚动。连续移动时，每批选择一个 Invoker 的一级窗口沿一个轴滚动，优先处理 LOD 0–2。后台只遍历进出窗口的切片与受影响的 2:1 接缝。粗层切片会跳过已定型的细层子树，常量区域的过渡节点在受影响处局部合并。渲染只重传受影响的空间分组。状态栏显示最近一次数据更新的耗时和重建块数。瞬移超出已发布 LOD0 时会显示 `LOD0 pending`，直到后台发布覆盖该位置的数据。

### LOD 滚动实测

Windows Release 下使用 VoxelField、`0.125×`（6.25 米）、512 米 LOD0 半径与逐级翻倍的范围。两名 Invoker 和跟随摄像机按同一路径连续移动 22 秒。LOD 更新耗时从提交后台请求计至体素与碰撞块发布，峰值只统计实际重建了分块的批次。

| 22 秒连续移动 | 单批耗时峰值 | 单批重建块数峰值 | 任一 Invoker 缺少已发布 LOD0 的采样帧 |
| --- | ---: | ---: | ---: |
| 滚动修复前 | 4,764 ms | 11,442 | 164 / 712 |
| 单轴切片滚动与局部合并 | 156 ms | 2,291 | 0 / 710 |

同配置的 DX12 Viewer 实测包含从轨道视角切到地表，单批峰值为 224 ms，两名 Invoker 在 1,823 帧内均处于已发布的 LOD0。首次构建、密度切换和视角跃迁的覆盖变化量与连续小步滚动不同。

### 在 Windows 上重新构建

安装 Visual Studio 2022 或 2026，选择「使用 C++ 的桌面开发」、Windows SDK 和 CMake。在项目根目录的 PowerShell 中运行构建脚本。首次构建会下载并校验依赖源码，准备 Debug 和 Release 各自链接的库，并生成 Win64 Visual Studio 解决方案。脚本默认编译 Release。添加 `-Test` 会对所选配置执行本机 CTest。 [Visual Studio C++ 安装说明](https://learn.microsoft.com/en-us/cpp/build/vscpp-step-0-installation)

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\build-windows.ps1
.\build-windows-native\dist\VoxelWorldViewer.exe
.\build-windows-native\dist\voxel_world.exe --scheme all --out results-windows
```

要构建 Debug 并生成 PDB，运行下面的命令。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\build-windows.ps1 -Configuration Debug -Test
```

在 Visual Studio 中打开 `build-windows-native` 下的 `VoxelWorld.sln` 或 `VoxelWorld.slnx`，选择 **Debug | x64**，确认启动项目为 **VoxelWorldViewer**，按 **F5**。调试命令行基准时，把 **voxel_world** 设为启动项目，可在项目的调试属性中填入 `--self-test`。打包后的 Debug 可执行文件和 PDB 位于 `build-windows-native\dist-debug`，Release 仍输出到 `build-windows-native\dist`。

`--self-test` 用局部地形验证异步密度切换、RayCast、ShapeCast 和刚体状态。Release 的 CTest 还会运行 `--self-test-global`，验证五档密度的全星球覆盖、远端碰撞、各级范围翻倍、单窗口单轴滚动与全量重建的布局一致性、LOD 接缝和全局切换。如需在 Debug 中复查这些耗时场景，可以直接向 `voxel_world.exe` 传入 `--self-test-global`。Viewer 的 `--capture-dir` 还会验证 Invoker 数量变更、相机接管、移动与脱离后回归，并保存 `invoker-control.bmp`。

### 浏览器预览

双击生成的 `VoxelWorld.html` 可离线查看同一份覆盖全星球的多层 Marching Cubes 网格。支持全星球、地表、Invoker 位置切换、LOD 着色和窗口线框。线框使用 OBJ 导出的窗口坐标，包括当时的摄像机 Invoker。此预览展示导出时的地形快照，实时碰撞、动态块更新和 df64 大气由原生查看器执行。预览保留 double 坐标，通过高低位顶点和相机相对消减显示，避免旧的 Int16 格式在地球尺度下溢出。

```bash
./build/voxel_world --scheme mesh --global --frames 1 --warmup 0 --bodies 0 --obj --out results-windows-preview
python scripts/make-preview.py results-windows-preview/mesh.obj results-windows-preview/VoxelWorld.html
python scripts/validate-lod-mesh.py results-windows-preview/mesh.obj
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

- `--invokers N` 控制回放的演员 Invoker 数量，默认两条不同路径，CLI 另加一条摄像机路径。全局模式按各源的距离范围从粗到细递归划分；多个源共用叶节点，相邻采样间距至多相差 2 倍。LOD 0–3 在默认 1,600 米的块内加密，块尺寸随精度倍率缩放；LOD 4 起扩大块，直至覆盖整个字段。原生状态栏显示 LOD 0/1/2/3/4+ 块数。局部基准模式继续使用四个有限窗口。
- `--stream-every N` 指定多少帧更新一次块覆盖，默认每六帧。局部模式用固定大小的块更新进入和离开窗口的切片；全局模式只重选穿过变化切片的八叉树分支，并局部平衡。归属变动和接缝受影响的块在构建完成后一起发布。发布前继续使用旧树、碰撞块和线框。CLI 使用同步构建来测量完整更新成本。
- `--frames`、`--queries`、`--bodies`、`--seed` 和 `--warmup` 可调。默认每帧每类 12 次查询，48 个动态刚体，90 个测量帧。无窗口基准逐个运行四个独立物理世界，按同样的顺序重放输入。查询与物理步进串行执行，地形分块并行构建。查询只包含地形。
- `summary.csv` 包含四类查询的 P50/P95/P99、命中、未判定、与网格方案的分歧及共同命中的 TOI 差值。`vs_first_field_different` 逐条比较字段后端的状态、初始重叠和 TOI。`frames.csv` 包含各层块数、移动扫描和构建阶段。`streaming.csv` 分开统计有块变更、无块变更的调度，并列出字段、碰撞体、CPU 渲染网格、调度元数据与新旧块并存时的内存估算。实际进程峰值 RSS 可用 `/usr/bin/time -f %M` 按后端单独测量。

### 胶囊 ShapeCast 压力测试

[查询性能报告](BENCHMARK.md) 包含默认回放、全星球和多种子压力组的实测结果。

```bash
./build/voxel_world --scheme all --capsule-stress --frames 90 --queries 30 --bodies 0 --out results-capsule-opt
```

`capsule-cases.csv` 按倾斜、贴地擦边、跨块长扫掠、明确未命中和起始重叠五类报告延迟、命中、未判定和相对网格的差异。体素字段含有实体内部信息，起始重叠会返回 `t=0`。网格基线采用 Bullet 的三角网格 sweep 回调，起点重叠后向外移动的返回语义可能不同。对比时可单独查看 `initial_overlap` 和 `grazing` 行。`--self-test` 另用解析平面验证倾斜长胶囊、切触、静止重叠、跨块 TOI 和批量角点读取。

## 原型边界

此项目输出渲染网格文件，并通过独立 CLI 测量地形碰撞。混合 LOD 使用边界 SDF 插值，并把细网格边界折线贴到无歧义的粗网格轮廓上。交错符号的鞍形面仍可能有缝隙。碰撞覆盖以当前驻留的块为准，超出加载范围的查询结果只能解释为当前数据的结果。ShapeCast 固定朝向，时间和空间细分有预算，返回 `Unresolved` 时应把该次计为未判定。刚体使用离散接触，速度过高或特征小于采样间距时可能穿透。CCD、编辑挖掘和原生 UE 集成超出本次基准范围。

Marching Cubes 查表数据来自 VTK 9.3.1，许可见 [VTK-Copyright.txt](third_party/VTK-Copyright.txt)。
