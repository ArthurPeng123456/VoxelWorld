# VoxelWorld

一个独立的 C++17 / Bullet3 星球碰撞实验。方案 1 按第三方插件的工作方式搭建基线。四种方案共享带撞击坑、坑缘和起伏的程序化星球，基准半径为 **6,371,000 米（地球尺度）**。种子控制地形和查询回放。场值是沿星球径向测得的有符号高度差，形状查询按三线性场的零等值面计算。

在 Windows 上运行 `scripts\build-windows.ps1` 构建原生 DX12 查看器，再打开 `build-windows-native\dist\VoxelWorldViewer.exe`。按 `F` 切换地表视角，按 `K` 显示各 Invoker 的数据范围。完整命令见[在 Windows 上重新构建](#在-windows-上重新构建)。

## 大世界坐标与大气

CPU 的 `Vec`、相机、网格顶点、变换、LOD 和碰撞查询均使用 double；Bullet 源码也以 `BT_USE_DOUBLE_PRECISION` 编译。使用 `VOXEL_SYSTEM_BULLET=ON` 时，必须提供以同一宏编译的双精度 SDK。体素存储中的 float 是局部有符号距离值，不承载世界坐标。

原生 GPU 上传边界将 double 拆成 `hi=float(value)`、`lo=float(value-double(hi))`。顶点着色器用 df64 先减去摄像机高低位，再转为 float 做相机相对空间投影，参考 [UE Large World Coordinates 的 DoubleFloat 和 translated world space](https://dev.epicgames.com/documentation/en-us/unreal-engine/large-world-coordinates-rendering-in-unreal-engine-5)。地形、刚体、标记和窗口线框共用此链路。深度使用 reverse Z，避免地球尺度远平面压缩近处深度精度。

`src/shaders/df64.hlsli` 实现 Andrew Thall《Extended-Precision Floating-Point Numbers for GPU Computation》的 Two-Sum、Fast-Two-Sum、4097 Split、Two-Product，以及 df64 加减乘除、校正平方根。所有误差变换使用 HLSL `precise`，编译启用 IEEE strictness，避免乘加融合或重排破坏误差项。采用论文的双 float 分支（约 48 位有效数字、float 指数范围），没有实现四 float 的完整 qf128 数学库，也不依赖硬件 FP64。

`src/shaders/atmosphere.hlsl` 用 df64 求大气射线区间和起点高度，然后沿局部射线求半径平方差，由 `(r² - R²) / (r + R)` 求各积分点的高度。大气默认厚度为 100 公里，采用 Rayleigh/Mie 单次散射、太阳遮挡、视线消光和地形空气透视。每条视线使用 16 个积分样本，每个样本使用 6 个太阳光路径样本。相机与星心先在 CPU double 中相减再拆分，支持带巨大公共平移的行星场景。 [大气散射的相函数和积分关系](https://ebruneton.github.io/precomputed_atmospheric_scattering/atmosphere/functions.glsl)

体素场使用八叉树叶节点覆盖整个星球的包围立方体，当前边界为 ±8,388,608 米，边长 16,777,216 米。每个 Invoker 和摄像机细分同一棵树，重叠区域保留最细层。默认 LOD0 采样间距为 **1 米**，可选 0.125 / 0.25 / 0.5 / 1 / 2 米。全局模式每个采样块固定每边 32 个单元，LOD 每增一级，采样间隔和块边长同时翻倍；相邻块保持 2:1 接缝。纯空气或实心区域通过保守的地形梯度界限认证后保留粗节点，块内常量场和单元行掩码也使用常量存储。

相邻叶节点在面、边、角保持 2:1 采样间距平衡，边界场值从粗层一致插值，细网格边界贴合对应粗 Marching Cubes 单元的轮廓。四种后端都从这些叶节点生成网格，粗层同样参与地形查询与碰撞。原生查看器默认启用此全局分层覆盖；CLI 使用 `--global` 启用，未指定时保留局部窗口基准模式。`--global --obj` 导出的全部三角形均来自体素场，不再附加远景球网格。天空着色器只负责大气，不再绘制或写入替代地面的解析球面；线框模式先写实体网格深度，避免背面线条穿透。地形起伏、撞击坑和 Invoker 移动仍保持原来的米级空间尺度。

Release 验证命令如下。`voxel_gpu_precision` 默认在 WARP 上运行相同的发布版 HLSL，也可用 `--hardware` 验证实际显卡。

```powershell
ctest --test-dir build-windows-native -C Release --output-on-failure
.\build-windows-native\Release\voxel_gpu_precision.exe --hardware
.\build-windows-native\Release\VoxelWorldViewer.exe --capture-dir results-earth-captures
.\build-windows-native\Release\VoxelWorldViewer.exe --warp --capture-dir results-earth-warp
```

GPU 测试包含 166 组数据，覆盖 df64 运算、毫米高度、地平线和云层球壳求交、球面风往返、跨相机云密度采样、轨道距离、10⁹/10¹² 米绝对坐标的相对消减，以及大气输出有限性和透射率范围。CPU 测试包含正负百万米坐标的射线、形状扫掠和刚体静止接触。截图验证覆盖四种后端的轨道、地表、高空地平线、线框与 LOD 窗口，另生成公共平移 `(10¹², -10¹², 10¹²)` 米前后的两张对照图。用 `python scripts/validate-render-captures.py results-earth-captures` 检查对照图。另有全局体积覆盖、32 个方向的远处地形查询、2:1 平衡、粗细块字段连续性，以及摄像机跨半球时的后台替换验证。大气采用单次散射，远处地形按粗层采样。

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

原生查看器的 RGBA8 G-buffer 保存地形高度、坡度与坑缘材质权重、其他物体的线性材质颜色及材质标记。RGBA16F G-buffer 保存世界法线与粗糙度，RGBA32F 保存前帧屏幕位置、前帧视图深度和材质标记，另有可读取的 D32 反向深度。全屏光照阶段从深度重建相机相对位置，计算方向光、环境光、镜面高光、级联阴影和大气空气透视。天空和调试线框也在线性 HDR 空间合成，经过 TAA 后统一处理曝光、ACES filmic 近似曲线和 sRGB 输出转换。场景颜色使用 `R16G16B16A16_FLOAT`，可以保留大于 1 的亮度；当前显示输出为色调映射后的 sRGB/SDR。[HDR 渲染参考](https://github.com/microsoft/DirectXTK12/wiki/Using-HDR-rendering) · [色调映射曲线来源](https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/)

地形着色使用程序化海拔与其梯度生成的平滑法线。随着网格采样间距和每像素覆盖面积增大，细节逐步过渡到星球径向法线和基准颜色。远景大气光程也平滑过渡到参考球面深度。截图验证在关闭大气和阴影的轨道画面中检查环纹及块边界。

CSM 使用四张深度图，默认每张 1024×1024，按对数与均匀混合方式划分视锥。CPU 使用 double 拟合包围球并在星球坐标系内对齐阴影纹素，GPU 继续在相机相对坐标系计算阴影。接收面使用法线偏移，3×3 PCF 过滤阴影边界，级联末尾 10% 混合到下一层。默认阴影距离随相机高度在 10–200 公里之间调整，末级向无阴影光照渐变，轨道视角超出局部阴影距离时关闭局部 CSM。[CSM 参考](https://learn.microsoft.com/en-us/windows/win32/dxtecharts/cascaded-shadow-maps)

`Shadows [J]` 或 J 切换阴影，`[` / `]` 每次降低／提高 0.5 EV。命令行 `--exposure EV` 接受 -10 到 10，默认 0；`--no-shadows` 关闭 CSM。例：

```powershell
.\build-windows-native\dist\VoxelWorldViewer.exe --exposure -0.5
.\build-windows-native\dist\VoxelWorldViewer.exe --no-shadows
```

`--capture-dir` 在可用时启用 D3D12 调试层，并采集阴影开关、三档分辨率、曝光 +1 EV、色调映射、大气调整与窗口缩放的固定场景。`.pipeline.txt` 记录 G-buffer 有效像素数、HDR 浮点最大值、阴影贴图实际分辨率和四个级联的深度像素数。`python scripts/validate-render-captures.py DIRECTORY` 检查画面对照和缓冲报告。GPU 数值测试还对照 CPU 检查曝光与色调映射结果。

### 行星体积云原型

默认开启球面体积云，工具栏 `Clouds [V]` 或 `V` 切换。自由摄像机可以从轨道连续飞到云下、云上和云内。`Render settings...` 中可调整云底高度、厚度、天气覆盖、消光强度、天气区尺度、云团尺度、风速及 32 / 64 / 96 步质量。

云场采用分层尺度。默认 1,400 公里级的连续球面天气分布决定云区与晴空，45 公里级的三维场塑造云团。约 1.4 km、370 m、155 m 的 Worley 频带只修改边缘的部分密度，扣除噪声均值后逐级衰减。每点按半分辨率像素的投影宽度和射线步长选择连续 mip，近处缩小采样点抖动幅度。天气、主体和细节使用统一的行星空间球面切向风，云历史按同一旋转回溯到前帧。

GPU 使用 df64 求视线球壳区间及积分锚点，保留空心云层的两个交段。太阳光也按壳内有效区间累积消光，逐点查询天气和云密度。每个连续路径在着色点附近密集采样，随后沿阳光方向扩大步长和过滤足迹，并在光学深度足够大时提前结束。云先以半分辨率计算 Beer–Lambert 消光、前后向散射和经验多次散射补光，再经独立时域重建和深度感知上采样合成到线性 HDR。地形接收低频天气阴影，前景不透明物体遮挡云。64³ RGBA 噪声含完整 mip，约 1.14 MiB。

低分辨率深度过滤和有限步数在复杂遮挡与掠射角仍有误差。多次散射、地面云影和大气耦合采用实时近似。[CIG 的轨道云报告](https://robertsspaceindustries.com/en/comm-link/transmission/18251-Star-Citizen-Monthly-Report-July-2021) · [Guerrilla 的天气与噪声建模](https://www.guerrilla-games.com/read/nubis-authoring-real-time-volumetric-cloudscapes-with-the-decima-engine)

命令行支持 `--clouds` / `--no-clouds`、`--cloud-base-km`、`--cloud-thickness-km`、`--cloud-coverage`、`--cloud-density`、`--cloud-weather-km`、`--cloud-formation-km`、`--cloud-wind-speed`、`--cloud-steps 32|64|96`。例如：

```powershell
.\build-windows-native\dist\VoxelWorldViewer.exe --terrain mars --exposure -1 --cloud-coverage 0.5 --cloud-weather-km 1400
.\build-windows-native\dist\VoxelWorldViewer.exe --capture-clouds results-clouds
python scripts/validate-cloud-captures.py results-clouds
```

`--capture-clouds` 生成轨道开关、零覆盖、零消光、两档采样质量、云下、地平线、云内、风、10¹² 米公共平移和前景遮挡共 14 个视图，以及云的不透明度图和 GPU 缓冲检查报告。`--capture-dir` 与 `--capture-clouds` 均关闭 TAA。

`Render settings... → Clouds` 中的 `Fine detail strength` 和 `Self-shadow strength` 默认均为 1，可设为 0 做对照。命令行分别是 `--cloud-detail` 和 `--cloud-self-shadow`。以下命令播放约 16 秒轨道往返航线，或者冻结风和曝光生成逐帧对照及 D3D12 GPU 时间戳。验证脚本依赖 NumPy。

```powershell
.\build-windows-native\dist\VoxelWorldViewer.exe --terrain mars --exposure -1 --replay-cloud-flight
.\build-windows-native\dist\VoxelWorldViewer.exe --terrain mars --exposure -1 --capture-cloud-flight results-cloud-flight
python scripts/validate-cloud-flight.py results-cloud-flight
```

回放记录五种配置各 209 帧。GPU 时间分为云积分和云重建，后者包含无效历史触发的补采样。对照固定轨道和云内镜头，以及同路线的全采样结果检测时序噪点。两组太阳方向截图验证自阴影改变光照且保持不透明度。原始数据保存在输出目录的 `cloud-flight.csv`，指标汇总在 `cloud-flight-metrics.json`。

### TAA 与体积云时域重建

默认开启 TAA，流程为 G-buffer → 云积分与云时域重建 → 延迟 HDR 光照 → 场景 TAA → 输出锐化、曝光与色调映射。投影采用 Halton(2,3) 亚像素抖动，可选择 8 / 16 / 32 帧周期，默认 16 帧、1 像素范围。场景历史在未曝光的线性 HDR 空间累积，使用深度和材质拒绝、YCoCg 邻域与方差裁剪、亮度加权混合，以及随运动降低的历史权重。锐化仅作用于最终输出，不反馈到历史。

场景 TAA 的输出与历史统一保存在不抖动的像素网格上：用当前投影偏移重建抖动后的 HDR 输入，再从运动数据中扣除上一帧投影偏移，并补偿选中深度样本的亚像素位置。天空也使用同一套坐标约定。直接把抖动后的栅格颜色输出到固定屏幕，会使整个画面跟随 Halton 序列高频晃动，即使历史重投影本身是正确的。这里保留投影抖动的抗锯齿采样作用，在重建阶段去掉它对最终画面位置的影响。

地形重投影使用前后帧相机基向量和 CPU double 计算后拆分的行星相对位移，避免把巨大世界坐标转成 float。刚体和 Invoker 标记通过稳定标识及顶点序号保存上一帧位置，包含自身平移与旋转；新增物体或拓扑变化不会继承无效运动。天空使用方向重投影，调试线框作为高响应覆盖层。切换场景、修改渲染参数、窗口缩放、相机瞬移或大角度转向都会清空历史；正常地形流式发布通过逐像素深度与材质检查处理。

云具有独立的半分辨率双缓冲历史，按贡献加权平均深度重投影，并补偿局部云形的风位移。采样方式可选全采样、二相棋盘格（每帧约 1/2 像素）或四相 2×2（约 1/4 像素），默认二相。相位在周期之间轮换，避免长期只使用 Halton 序列的一个子集；积分点随机偏移随时间变化，由历史累积降低条带和噪声。历史越界、遮挡变化、无可用邻域或冷启动时，缺失像素当帧补做完整射线积分。关闭 TAA 或云时域重建会自动回到全采样，并关闭云射线的时间抖动。最终场景 TAA 根据云不透明度降低二次累积，减少与地面运动不同的云拖影。

稀疏云只对当帧真正积分过的像素做当前邻域裁剪。未采样像素保留通过深度、遮挡及风重投影检查的自身历史，避免把四相模式下仅有一个有效邻居的退化裁剪框当成当前像素颜色，导致云图案随相位跳动。无效历史仍即时补采样。

参考：[Karis 的 High Quality Temporal Supersampling](https://www.advances.realtimerendering.com/s2014/index.html)、[Salvi 的 Temporal Supersampling 与方差裁剪](https://developer.download.nvidia.com/gameworks/events/GDC2016/msalvi_temporal_supersampling.pdf)。当前云重投影用单一平均深度近似整个体积，复杂多层云、快速移动与掠射角仍可能有误差。提高历史权重可降低闪烁，也可能增加残影；降低历史权重、提高 Motion response 可以提高运动响应。TAA 对极细特征可能产生软化，可调整裁剪、历史和输出锐化。

TAA 的运动目标和场景历史合计约为每个全分辨率像素 48 字节，云历史约为 12 字节，总计在 1920×1080 下约 119 MiB（不含资源对齐、原有 G-buffer 和云原始目标）。这些资源随窗口尺寸预分配；关闭时域过滤会跳过相关渲染与棋盘格流程，但保留分配。

```powershell
.\build-windows-native\dist\VoxelWorldViewer.exe --taa --taa-samples 16 --taa-history 0.9 --taa-sharpen 0.15 --cloud-sampling checkerboard
.\build-windows-native\dist\VoxelWorldViewer.exe --cloud-sampling quarter --cloud-history 0.92 --cloud-ray-jitter 0.85
.\build-windows-native\dist\VoxelWorldViewer.exe --no-taa
.\build-windows-native\dist\VoxelWorldViewer.exe --capture-temporal results-temporal
python scripts/validate-temporal-captures.py results-temporal
.\build-windows-native\dist\VoxelWorldViewer.exe --warp --capture-temporal results-temporal-warp
python scripts/validate-temporal-captures.py results-temporal-warp
```

`--capture-temporal` 生成 36 个视图及 GPU 报告。它用 16 个独立抖动样本的线性 HDR 均值验证静态边缘误差，检查移动刚体的运动数据与色彩拖影、平移视角、相机切换、窗口缩放、10¹² 米公共平移、云采样档位、关闭／重启 TAA、前景遮挡出现和移除后的补采样，以及风驱动重投影。测试也输出中央 256² 像素的原始与累积 HDR 数据，避免仅凭色调映射后的截图判断抗锯齿。修正输出网格后，专项场景边缘 RMSE 约降低 63%，移动目标未检测到长色彩拖影，公共平移基准逐像素一致。轨道测试充分预热后，二相与四相云的主射线积分步数约为全采样的 50% 和 26%；这衡量主射线工作量，不等同于整个渲染器的耗时或帧率倍率。GPU 精度测试还编译全部 10 个发布版着色器入口。

连续帧稳定性使用另一组固定机位、冻结模拟时间的场景验证。每组预热 64 帧，再采集 32 帧，覆盖无云轨道、全／二相云轨道、二相／四相云地平线，以及关闭投影抖动或云积分抖动的对照组。脚本检查历史连续有效、GPU 数值有限性和画面亮度的帧间二阶差分 RMS，避免单张截图通过但连续画面抖动。相对修复前的同机位结果，无云场景高频变化从 6.020 降到 0.229（约减少 96%），默认二相云轨道从 6.266 降到 0.788（约减少 87%），四相云地平线从 0.772 降到 0.140（约减少 82%）；数值单位为 8 位显示亮度级，属于这些固定测试场景的结果。

```powershell
.\build-windows-native\dist\VoxelWorldViewer.exe --capture-stability results-stability
python scripts/validate-temporal-stability.py results-stability
.\build-windows-native\dist\VoxelWorldViewer.exe --warp --capture-stability results-stability-warp
python scripts/validate-temporal-stability.py results-stability-warp
# 可选：与保留的修复前连续帧目录比较
python scripts/validate-temporal-stability.py results-stability --baseline results-stability-before
```

### 渲染设置

工具栏 **`Render settings...`** 打开包含 **CSM / Atmosphere / Clouds / TAA / HDR** 五个 tab 页的设置窗口。切换页签保留尚未应用的输入；点击 `Apply` 验证全部页面、应用到当前场景并清空时域历史，`Defaults` 恢复各页初始参数，`Cancel` 放弃本次编辑。`J`、`H`、`V` 和 `[` / `]` 继续快速切换阴影、大气、云和曝光。

| 分组 | 可调参数 |
| --- | --- |
| CSM | 开关、每级 Shadow Map 分辨率（1024² / 2048² / 4096²）、距离上限、对数分割权重、法线和深度偏移倍率 |
| 大气 | 开关、厚度、Rayleigh / Mie 各自的尺度高度与散射强度、Mie 各向异性 |
| 体积云 | 开关、云底、厚度、覆盖、消光、天气与云团尺度、风速、细节与自阴影强度、采样质量、时域重建开关、全／二相／四相采样、历史权重、深度拒绝、射线抖动 |
| TAA | 开关、8 / 16 / 32 帧抖动周期、历史权重、抖动范围、方差裁剪 gamma、深度拒绝、运动响应、输出锐化、相机切换的距离与角度阈值 |
| HDR | 曝光（−10 到 +10 EV）和色调映射（ACES、Reinhard、线性裁剪） |

`Thickness (km)` 同时调整大气外边界和密度曲线的垂直尺度。Rayleigh 与 Mie 的尺度高度按 100 公里厚度填写，运行时随厚度等比缩放。例如默认 Rayleigh 尺度高度为 8 公里，厚度调到 200 公里时对应的实际尺度高度为 16 公里。轨道截图验证固定机位，只切换 100 公里与 300 公里厚度，检查外缘光晕的变化。

四张 D32 阴影贴图的显存用量分别约为 16、64、256 MiB。切换分辨率时会重建阴影资源。HDR 场景仍通过色调映射输出到 sRGB/SDR 交换链。

命令行可用 `--shadow-resolution`、`--shadow-distance-km`、`--shadow-split-blend`、`--shadow-normal-bias`、`--shadow-depth-bias` 设置 CSM。大气对应 `--atmosphere-height-km`、`--rayleigh-height-km`、`--mie-height-km`、`--rayleigh-strength`、`--mie-strength`、`--mie-anisotropy` 和 `--no-atmosphere`。HDR 使用 `--exposure` 和 `--tone-map aces|reinhard|linear`。

TAA 使用 `--taa` / `--no-taa`、`--taa-samples 8|16|32`、`--taa-history`、`--taa-jitter`、`--taa-variance`、`--taa-depth-threshold`、`--taa-motion-response`、`--taa-sharpen`、`--taa-cut-distance-km`、`--taa-cut-angle`。云时域参数为 `--cloud-temporal` / `--no-cloud-temporal`、`--cloud-sampling full|checkerboard|quarter`、`--cloud-history`、`--cloud-depth-threshold`、`--cloud-ray-jitter`。

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
| 设置 Invoker 数量、巡航模式与移动速度 | 工具栏 `Invoker settings...` |
| 跟随指定 Invoker（继续自动巡航，WASD 接管） | 工具栏 `Camera` 选择 `Invoker N`，或按 `Tab` 切到下一个 |
| 接管时沿地表移动、加速 | `W` / `A` / `S` / `D`，`Shift`（3 倍） |
| 调整受控 Invoker 的基础移动速度 | 小键盘 `+` / `-`（乘／除 1.25） |
| 脱离当前 Invoker | `Esc` 或 `Camera` 选择 `Detached`，也可用 `C` 接入自由相机 |
| 开关 LOD 着色 | `L` |
| 开关各 Invoker 的 3D LOD 窗口线框 | `K` |
| 设置 LOD0 每边单元数并预览各级米数 | 工具栏 `LOD ranges...` |
| 切换地形和刚体的三角形线框渲染 | `F2` 或 `Wireframe [F2]` |
| 开关大气散射和地形空气透视（默认开启） | `H` 或 `Atmosphere [H]` |
| 开关行星体积云（默认开启） | `V` 或 `Clouds [V]` |
| 开关 CSM 阴影（默认开启） | `J` 或 `Shadows [J]` |
| 调整 CSM、大气、云、TAA 和 HDR | 工具栏 `Render settings...` 的对应 tab 页 |
| 选择星球地形和材质 | 工具栏 `Terrain` |
| 降低／提高曝光 0.5 EV | `[` / `]` |
| 向当前 Invoker 附近投放一组球、胶囊和盒子 | `B` |
| 暂停或重置模拟 | `Space`、`R` |

自由摄像机以 double 保存位置，W/S 沿视线前后移动，A/D 左右平移，Q/E 沿相机上方向升降。初始速度随进入时的高度调整，状态栏显示基础速度；滚轮向上加速、向下减速，Shift 临时加速 5 倍。松开移动键或切换到其他应用时停止移动，暂停模拟仍可飞行并更新地形 LOD。该模式不做地形碰撞约束。再次按 C 返回先前的轨道／地表模式，F 切换至锚定视角，Reset 退出自由模式。工具栏会随窗口宽度自动换行。

右下角的 `Camera world (m)` 面板显示当前渲染摄像机的世界坐标 X / Y / Z、相机模式和 `Alt (sphere)` 高度。数值单位为米，保留三位小数；高度相对半径 6,371,000 米的参考球面计算。面板随每帧摄像机位置刷新，暂停模拟时仍会更新自由摄像机信息。

`Invoker settings...` 打开独立设置面板，数量选项已从主工具栏移入面板。设置统一应用到所有 Invoker，默认数量 2，支持手动输入 1–10000 的整数。摄像机另外参与数据 LOD，工具栏的 `Camera` 用于跟随目标，WASD 开始手动接管。

- **Area patrol**：在指定半径内沿地表往返巡航，默认半径 1792 米；这是巡航活动范围，与体素 LOD range 无关。
- **Great-circle cruise**：沿穿过星心的固定大圆平面持续前进，贴合程序化地表高度，可以跨越经线、半球并绕行星球。
- **Cruise speed**：巡航速度，默认 1.5 米/秒。范围模式在转向处自然减速，大圆模式持续前进。
- **Return speed**：脱离控制后的归航速度，默认 3 米/秒。
- **Controlled speed**：受控移动的基础速度，默认 4.5 米/秒；小键盘 `+` 乘以 1.25，`-` 除以 1.25，状态栏即时显示，设置面板同步显示当前值。按住 Shift 临时加速 3 倍。

巡航和归航速度可设为 0 以暂停对应移动。Camera 选择目标后只跟随，不会暂停自动巡航。实际按 WASD 后才进入手动控制并暂停该 Invoker 的路线；Esc 脱离后按归航速度回到暂停的巡航点，再继续巡航；归航速度低于巡航速度时也能归队。速度修改保留路线进度；修改巡航模式或半径会从当前位置和当前前进方向建立新路线，保持位置连续，并恢复目标的自主巡航。调整数量保留现有刚体和其余 Invoker，移除当前受控目标时会安全脱离。暂停模拟仍可手动移动，自动巡航和归航等待模拟恢复。切换后端和 Reset 保留所选参数。Apply 生效，Cancel 丢弃本次编辑，Defaults 恢复上述默认值。

查看器也支持命令行设置：

```powershell
.\build-windows-native\dist\VoxelWorldViewer.exe --invokers 2 --invoker-cruise great-circle --invoker-cruise-speed 1.5 --invoker-return-speed 3 --invoker-control-speed 4.5
.\build-windows-native\dist\VoxelWorldViewer.exe --invoker-cruise area --invoker-patrol-radius-m 100
```

`--invokers N` 在查看器和基准程序中也接受 1–10000。巡航和归航状态按实际数量动态分配。

`--capture-invokers DIRECTORY` 检查增加到 10000、接管第 10000 个目标并安全缩减、两种模式切换、巡航限速、接管暂停路线、归航完成以及小键盘调速；可以配合上述参数测试自定义速度，包括归航慢于巡航的配置。数量测试在地形更新前恢复小规模场景，不代表 10000 个 Invoker 同时细化地形的性能压测。CPU `--self-test` 另外验证整数输入上下界、范围边界、大圆完整绕行和固定平面、跨半球贴地移动、零速度暂停及参数校验。

Invoker 初始方向使用与数量无关的低差异球面序列，并以对跖点成对分布，覆盖整个星球；不会重复使用有限的撞击坑中心。先放在参考球面，以解析地形位置引导 LOD 请求，已发布的体素网格就绪后再沿径向投射到可见三角形上方 5 厘米。巡航和手动移动会持续贴地，LOD 替换时重新校正高度。地形尚未就绪的标记暂不显示，状态栏标记 `Surface pending`。接地查询缓存所属块和支撑三角形，只在离开该三角形或块被替换后重新查找。

滚动 clipmap 按各个 Invoker（包括摄像机）独立选择待更新层级，每个源在同一批次最多滚动一个窗口，并合并所有脏区域后统一平衡、生成和发布；不会让 256 个巡航点排队共享一个窗口名额。新增 Invoker 在本地 LOD0 发布前等待，自动巡航和归行也会在即将离开已发布细网格时等待流式更新，保留路线进度并在地形就绪后自动继续。手动移动和自由摄像机保持直接响应。状态栏 `LOD0 waiting` 显示还没有细网格的 Invoker 数量；256 个全球细化区的初次生成仍可能消耗大量时间和内存。

测试刚体使用接近人体的米制尺寸：球体直径 1 米，胶囊体直径 1 米、总高 2 米（圆柱段高 1 米），箱体宽深各 1 米、高 2 米。显示几何和 Bullet 碰撞使用同一套尺寸。Invoker 的可视标记宽约 1 米、高 2 米，Invoker 本身仍是采样位置。投放物体分布在约 8.5 米半径内，从数米高度落下。地表概览相机默认距离 12 米，接管 Invoker 时默认跟随距离 6 米，视线对准离地 1 米的位置；滚轮可继续拉近，低空自由相机以 1 米/秒起步。

切换后端会从同一种子重建场景和刚体。状态栏显示 LOD 块数、三角形数量、刚体数量和 LOD 更新耗时，并每 0.25 秒刷新内存统计（MiB）：`RAM` 为当前进程工作集，`Private commit` 为进程私有提交量，`Peak RAM` 为进程启动以来的峰值工作集。另列出当前驻留地形的估算总量及体素字段、CPU 网格、碰撞、调度元数据分项；该估算属于进程内存的一部分，不包含后台构建、动态刚体、渲染资源和分配器额外开销，不应与进程统计相加。命令行可用 `--seed`、`--invokers`、`--scheme` 和 `--morton`。设备创建失败时会尝试 Direct3D 12 的 WARP 软件渲染，也可用 `--warp` 强制启用。受组织应用控制管理的电脑可能需要管理员批准本地生成的可执行文件。

### 地形与材质预设

`Terrain` 下拉菜单可在三套地形之间切换。切换时按当前种子重建地形、碰撞和刚体场景。也可在查看器或基准程序使用 `--terrain classic|mars|volcanic` 和 `--seed N`。

| 预设 | 地形 | 材质 |
| --- | --- | --- |
| Classic | 原有的球面起伏和撞击坑 | 原有的灰色高度与坡度着色 |
| Mars Craters | 全星球分布的撞击坑，从几十米级小坑到千公里级盆地，带坑缘、外侧抛射堆积和中央隆起 | 氧化铁红尘、深色坑缘、岩区和岩屑 |
| Volcanic Highlands | 山脊、熔岩沟槽、宽广火山口及周围起伏 | 深色玄武岩、灰褐火山灰和氧化岩区 |

Mars Craters 的默认大气厚度为 **300 km**，云底为 **6 km**、云层厚度为 **24 km**、天气覆盖为 **0.35**；其余大气与云参数使用设置窗口中的初始值，包括开启大气和云、64 步积分、二相棋盘格和云时域重建。启动 `--terrain mars`、在界面切换到 Mars Craters 或在该星球的渲染设置中点击 `Defaults` 都会使用对应默认值。命令行显式指定的渲染参数优先于启动预设，与参数顺序无关。切换其他星球时，大气厚度、云层厚度与覆盖率恢复各自预设值。

预设使用当前工程的星球尺度和 LOD。区域起伏和坑形进入四种后端共用的 SDF。新预设的完全空区和实心区在远离零面时压缩为常量块。两套新材质使用连续的球面色区和 64、16、4 米三档程序化纹理，随屏幕像素覆盖范围渐隐。4 米层包含材质法线起伏。F 从轨道切到 Invoker 附近，滚轮可拉近看颗粒和坑壁。

火星预设使用约 4,600 个显式大型撞击坑与五级按球面位置和种子实时计算的中小型撞击坑。小坑可叠加在较老盆地内部。体素 LOD 决定能解析的最小坑形，轨道视角保留可分辨的大盆地。火星截图验证另外访问两个远离初始 Invoker 的区域，采集近地画面并检查当地的碰撞。

### 体素网格精度

工具栏 `Voxel step` 的数值直接对应 **米**，默认 `1x (1 m)`；`0.5x` 就是 0.5 米，`2x` 就是 2 米。它们不再乘旧版的 50 米基准。体素单元是相邻采样点之间的间隔，N 个单元需要 N+1 个采样点。

| 设置 | LOD0 采样间距 | 全场逻辑网格每边单元数 | 默认 range=256 的名义边长 |
| --- | --- | --- | --- |
| `0.125` | 0.125 米 | 2²⁷ | 32 米 |
| `0.25` | 0.25 米 | 2²⁶ | 64 米 |
| `0.5` | 0.5 米 | 2²⁵ | 128 米 |
| `1`（默认） | 1 米 | 2²⁴ | 256 米 |
| `2` | 2 米 | 2²³ | 512 米 |

切换后在后台重建体素场、网格和碰撞，完成后统一替换；摄像机、刚体位置及速度继续保留。range 的单元数量保持不变，实际覆盖米数随间隔同步变化，星球尺寸和全场边界不变。状态栏显示已发布的间隔、单元数和实际边长，重建期间另显示目标值。切换后端和 Reset 会保留选择。

原生查看器和命令行基准都支持 `--voxel-density`（也接受 `--density`），其值现在是 LOD0 的米制采样间隔；`--lod0-range` 是全局 clipmap 的每边单元数。

```powershell
.\build-windows-native\dist\VoxelWorldViewer.exe --voxel-density 0.5 --lod0-range 256
.\build-windows-native\dist\voxel_world.exe --scheme grid --global --voxel-density 0.5 --lod0-range 64 --frames 1 --warmup 0 --bodies 0 --obj --out results-density
```

CLI 的 `settings.txt` 保存采样间隔、range 单元数、名义边长与半边长；OBJ 写入 `finest_spacing_m` 和 `lod0_cells_per_edge` 元数据。浏览器预览按实际米数显示，新打包脚本同时兼容旧版 50 米基准的 OBJ。

### LOD 范围

点击 `LOD ranges...`，填写 **LOD0 cells / edge**。默认 **256**，有效值为 32 至 4,194,304 的整数 2 的幂。这个值表示每条边的单元数，既不是米数，也不是半径。最小值对应一个 32³ 的存储块。

设 range 为 N，LOD0 间隔为 s 米，则 LOD k 的采样间隔为 `s × 2^k`，名义窗口边长为 `N × s × 2^k`，半边长是它的一半。例如：

| range=4096 | LOD0 间隔 | LOD0 名义边长 | LOD1 名义边长 |
| --- | --- | --- | --- |
| 0.125x | 0.125 米 | 512 米 | 1024 米 |
| 0.5x | 0.5 米 | 2048 米 | 4096 米 |
| 1x | 1 米 | 4096 米 | 8192 米 |
| 2x | 2 米 | 8192 米 | 16384 米 |

这是分辨率与覆盖的定义，不会预先密集分配 N³ 个体素：内层由更细的 LOD 占据，外层只保留相应的粗块，空区和实心区保持稀疏。每个 Invoker 有一组独立窗口。窗口按块滚动并保留滞回余量，实际驻留边界还会向块边界取整并进行 2:1 平衡，因此实际覆盖的单元数可能略多于 N，窗口米数也可能略大于名义边长。`K` 显示实际已发布窗口，`L` 显示实际地表 LOD；对话框显示各级名义间距和边长，以及 LOD0 的半边长。当窗口半边长足以从 Invoker 覆盖全场时，后续使用全场回退层。

全局 LOD 的砖块在每一级都保持 32³ 个单元，块宽随 LOD 放大，避免旧版远层只有每边 4 个单元导致的大量小块。`Planet::mayContainSurface` 使用保守的梯度界限判断可合并的纯空或实心区域，不仅检查角点；同一界限也用于字段采样前的常量认证。大范围依然有真实的开销：4096³ 是约 **687 亿个逻辑单元**，即使只有一张平面地表，每层也有约 **1678 万个地表单元**，再加网格、碰撞、多个 Invoker 与后台替换所需内存。默认值 256 用于常规交互，4096 需要根据内存和加载时间评估。

连续移动时，查看器根据各 Invoker 和相机速度预测窗口位置，每批为每个源独立滚动一个窗口和一个轴，处理进出窗口的切片及 2:1 接缝。顶点着色、GPU 打包和 Bullet 碰撞体在后台准备，GPU 按帧上传完成后统一发布，旧数据保留到替换就绪。更改范围同时更新全场回退层，防止远端留在旧分辨率。状态栏 `LOD blocks 0/1/2/3/4+` 是已发布的块数；`LOD0 256 cells/edge (128 m side)` 表示 0.5 米间隔下的已发布范围，重建时显示目标单元数与米数。暂停模拟仍可更新地形。

`--capture-lod-ranges DIRECTORY` 验证当前 range → 两倍 range → 原 range，保存四张对照图和 `lod-range-validation.txt`；报告同时记录采样间隔、名义边长、含余量的窗口边长及块数。可以使用 `--lod0-range 64` 快速检查发布过程，也可配合 `--terrain mars`、`--voxel-density 0.5` 检查其他地形和间隔。CPU 集成测试覆盖五档间隔、4096 单元的换算、全场覆盖、稀疏合并、相邻 LOD 接缝、滚动预测、运行时切换以及刚体状态保留。

### LOD 滚动实测

以下是旧版 50 米基准的历史测量，不能用作当前米制网格的性能结果。Windows Release 下使用 VoxelField、`0.125×`（旧版 6.25 米）、512 米 LOD0 半径与逐级翻倍的范围。两名 Invoker 和跟随摄像机按同一路径连续移动 22 秒。LOD 更新耗时从提交后台请求计至体素与碰撞块发布，峰值只统计实际重建了分块的批次。

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

`--self-test` 用局部地形验证异步密度切换、RayCast、ShapeCast 和刚体状态。Release 的 CTest 还会运行 `--self-test-global`，验证五档密度的全星球覆盖、远端碰撞、各级范围翻倍、各源独立滚动与全量重建的布局一致性、LOD 接缝和全局切换。如需在 Debug 中复查这些耗时场景，可以直接向 `voxel_world.exe` 传入 `--self-test-global`。Viewer 的 `--capture-dir` 还会验证 Invoker 数量变更、相机接管、移动与脱离后回归，并保存 `invoker-control.bmp`。

`voxel_clipmap_distribution.exe 256 256` 单独验证 256 个全球 Invoker、256 单元 range：运行时增加数量后的布局必须与全量重建一致，每个落点实际生成的细网格必须能接地，所有点同时离开旧窗口后也必须在同批恢复 LOD0，并与统一平衡后的全量布局一致。它逐点生成接地网格，不常驻全星球的全部字段和 GPU 顶点。Release 的 CTest 使用 `16 64` 快速执行同一组回归。

Viewer 的 `--capture-clipmaps DIRECTORY --invokers 8 --lod0-range 64` 验证增加数量时等待 LOD0、发布后恢复巡航、实际网格接地及各层三角形数量，保存 `clipmaps.csv` 和多处统一高度的 LOD 着色/线框俯视图。它会完整生成和上传请求的全部区域；大数量检查可先使用上述 CPU 验证程序。

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

- `--invokers N` 控制回放的演员 Invoker 数量，默认两条不同路径，CLI 另加一条摄像机路径。全局模式按各源的距离范围从粗到细递归划分；多个源共用叶节点，相邻采样间距至多相差 2 倍。全局 LOD 块固定每边 32 个单元，LOD0 的默认块边长为 32 米，块边长和间隔随 LOD 逐级翻倍，直至全场回退。原生状态栏显示 LOD 0/1/2/3/4+ 块数。局部基准模式继续使用四个有限窗口。
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
