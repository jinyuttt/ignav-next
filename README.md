# ignav-next

INS/GNSS 组合导航系统 — 基于接口契约的多模块 Maven 项目

## 项目起源

ignav-next 的算法核心源自 **IGNAV**（INS/GNSS Navigation）C 语言项目。IGNAV 是一个开源的 INS/GNSS 紧组合导航算法库，原始版本使用 C 语言编写，采用过程式编程风格，核心算法包括 INS 机械编排、EKF 松/紧组合、辅助约束（NHC/ZVU/ZARU/里程计）、RTS/FBS 平滑等。

将 C 版本移植为 Java 版本的原因：

1. **可维护性**：C 版本的全局状态和过程式风格导致代码耦合度高、难以扩展。Java 的面向对象特性天然适合模块化设计
2. **跨平台部署**：Java 一次编译到处运行，便于在 Android、嵌入式 Linux、服务器等平台部署
3. **生态优势**：Java 生态中 rtklib-java 提供 GNSS 解算能力，EJML 提供高效矩阵运算，SLF4J/Logback 提供结构化日志
4. **工程规范**：Maven 多模块 + 契约层架构，使 INS、GNSS、融合三层可独立开发、独立测试、独立演进

本项目在算法层面与 C 版本保持一致（相同的力学模型、相同的 EKF 状态定义、相同的辅助约束逻辑），但在架构层面进行了重新设计：

| 方面 | C 版本 (IGNAV) | Java 版本 (ignav-next) |
|------|---------------|----------------------|
| 语言 | C | Java 17 |
| 架构 | 过程式，全局状态 | 契约层 + 模块化 + 接口驱动 |
| 矩阵库 | 自定义 matmul/matinv | EJML (Efficient Java Matrix Library) |
| GNSS 解算 | RTKLIB C | rtklib-java |
| 构建系统 | Makefile | Maven 多模块 |
| 日志 | printf | SLF4J + Logback |
| 状态管理 | 全局 InsState 结构体 | InsProvider/InsState 封装 |
| 融合模式 | 编译时选择 | 运行时自适应切换 |
| 辅助约束 | 函数调用 | 独立 Aiding 类 + 策略模式 |
| 平滑 | 后处理脚本 | 内置 InsRts/InsFbs |

## 项目概述

ignav-next 是一个模块化的 INS/GNSS 组合导航系统，核心设计原则是 **INS 和 GNSS 作为两个独立演进的模块，通过 contract 层连接，融合层做主控协调双向数据流**。

## 模块结构

```
ignav-next/
├── ignav-contract/     # 共享接口和数据类型
├── ignav-ins/          # INS 惯导模块
├── ignav-gnss/         # GNSS 卫导模块
├── ignav-fusion/       # 融合层
└── ignav-app/          # 应用层
```

### 模块依赖关系

```
ignav-app → ignav-fusion → ignav-contract ← ignav-ins
                          → ignav-ins       ← ignav-gnss
                          → ignav-gnss
```

所有模块间的通信通过 `ignav-contract` 中定义的接口进行，模块之间无直接依赖。

## 技术栈

| 项目 | 版本 |
|------|------|
| Java | 17 |
| Maven | 3.9.x |
| EJML | 0.43.1 |
| rtklib-java | 2.0.5 |
| SLF4J | 2.0.9 |
| Logback | 1.4.8 |
| JUnit | 5.9.2 |

## 快速开始

```bash
# 编译
mvn compile

# 运行测试
mvn test

# 打包
mvn package
```

---

## ignav-contract — 契约层

定义模块间通信的接口和数据类型，是整个系统的"宪法"。

### 接口

#### InsProvider — INS 提供者接口

```java
public interface InsProvider {
    void timeUpdate(ImuMeasurement imu);           // IMU 数据驱动 INS 更新
    InsPrediction getPrediction();                  // 获取 INS 预测（供融合层使用）
    void applyCorrection(StateCorrection correction); // 应用融合层的状态修正
    InsSolution getSolution();                      // 获取 INS 导航解
    void configure(InsConfig config);               // 配置 INS 参数
    void setTimeProvider(TimeProvider timeProvider); // 注入时间同步提供者
    SystemHealth getHealth();                       // 获取 INS 健康状态
    int getSupportedContractVersion();              // 契约版本兼容性
}
```

#### GnssProvider — GNSS 提供者接口

```java
public interface GnssProvider {
    void setInsPrediction(InsPrediction prediction); // 接收 INS 预测（辅助 GNSS）
    GnssObservation computeObservation();            // 计算观测矩阵（紧组合用）
    GnssPositionSolution solvePosition();            // 求解 GNSS 位置
    void configure(GnssConfig config);               // 配置 GNSS 参数
    void setTimeProvider(TimeProvider timeProvider);  // 注入时间同步提供者
    SystemHealth getHealth();                        // 获取 GNSS 健康状态
    int getAvailableSatellites();                    // 可用卫星数
    double getGDOP();                                // 几何精度因子
    int getSupportedContractVersion();               // 契约版本兼容性
}
```

#### TimeProvider — 时间同步接口

时间同步是**融合层的职责**，INS 和 GNSS 只认系统时间，不做内部时间转换。

```java
public interface TimeProvider {
    GTime getCurrentTime();                  // 统一系统时间基准
    GTime imuToSystem(GTime imuTime);        // IMU 时间戳 → 系统时间
    GTime gnssToSystem(GTime gnssTime);      // GNSS 时间戳 → 系统时间
    void setTimeBias(double biasSec);        // 设置时间偏差（融合层估计）
    double getTimeBias();                    // 获取当前时间偏差
    double getImuTimeDrift();                // IMU 时钟漂移
    double getGnssTimeDrift();               // GNSS 时钟漂移
}
```

### 数据类型

| 类 | 说明 |
|----|------|
| `GTime` | GPS 时间（周内秒 + 整秒） |
| `ImuMeasurement` | IMU 测量数据（陀螺 + 加速度计） |
| `InsPrediction` | INS 预测结果（位置/速度/姿态 + 协方差） |
| `InsSolution` | INS 导航解（含状态标志） |
| `InsConfig` | INS 配置参数（噪声PSD、初始状态、辅助开关等） |
| `GnssObservation` | GNSS 观测量（新息向量 v、观测矩阵 H、噪声矩阵 R） |
| `GnssPositionSolution` | GNSS 位置解（含解状态、卫星数、Ratio 等） |
| `GnssConfig` | GNSS 配置参数 |
| `StateCorrection` | 状态修正量（dx + dP） |
| `SystemHealth` | 系统健康状态 |
| `FusionMode` | 融合模式及自适应切换逻辑 |
| `ContractVersion` | 契约版本管理 |

### 系统健康状态

```java
public enum HealthStatus {
    NOMINAL,        // 正常
    INS_ONLY,       // GNSS 中断，纯 INS
    GNSS_ONLY,      // INS 失效
    INS_DEGRADED,   // INS 精度降低
    GNSS_DEGRADED,  // GNSS 精度降低
    DEGRADED,       // 整体精度降低
    FAILED          // 系统失效
}
```

健康判断基于**协方差阈值**和**新息序列**，而非二元判断。

### 融合模式自适应切换

```java
public enum Mode {
    INS_ONLY,  // 纯 INS（卫星数 < 2）
    LC,        // 松组合（卫星数 ≥ 2, GDOP < 20）
    STC,       // 半紧组合（卫星数 ≥ 4, GDOP < 6）
    TC         // 紧组合（卫星数 ≥ 6, GDOP < 3）
}
```

切换逻辑包含：
- **滞回机制**（hysteresis）：在当前模式保持 10 个历元后才允许切换，避免边界抖动
- **冷却期**（cooldown）：切换后 30 个历元内不再切换
- **阈值可配置**：所有卫星数/GDOP 阈值均可通过 setter 调整

### 契约版本管理

```java
public final class ContractVersion {
    public static final int VERSION = 1;
    public static boolean isCompatible(int providerVersion) {
        return providerVersion >= 1 && providerVersion <= VERSION;
    }
}
```

版本兼容规则：`1 <= providerVersion <= VERSION`，保证向后兼容。

---

## ignav-ins — INS 惯导模块

### 包结构

```
org.gnss.ignav.ins/
├── InsProviderImpl.java    # InsProvider 契约实现
├── common/                 # 公共基础
│   ├── IgnavConstants.java # 系统常量（WGS84、GNSS频率、INS状态码等）
│   ├── InsMath.java        # 数学运算（矩阵、坐标变换、四元数等）
│   └── Quaternion.java     # 四元数运算
├── data/                   # 数据结构
│   ├── GTime.java          # GPS 时间
│   ├── Imud.java           # IMU 数据
│   ├── InsState.java       # INS 状态（位置/速度/姿态/误差状态）
│   ├── InsOpt.java         # INS 选项配置
│   ├── InsPsd.java         # 功率谱密度参数
│   ├── InsUnc.java         # 不确定性参数
│   ├── ImuErr.java         # IMU 误差模型参数
│   ├── InsAlign.java       # 对准参数
│   ├── InsZvOpt.java       # ZVU 选项
│   ├── Odopt.java          # 里程计选项
│   ├── MagOpt.java         # 磁力计选项
│   ├── Odod.java           # 里程计数据
│   └── Gmea.java/Gmeas.java # GNSS 测量数据
├── mech/                   # 机械编排
│   ├── InsMech.java        # 正向机械编排（ECEF/NED 两种框架）
│   ├── InsAlignMech.java   # 对准（粗对准 + 精对准 + 速度匹配）
│   └── InsBackMech.java    # 反向机械编排
├── aiding/                 # 辅助约束
│   ├── InsNhc.java         # 非完整约束（NHC）
│   ├── InsZvu.java         # 零速检测与更新（ZVU）
│   ├── InsZaru.java        # 零角速率更新（ZARU）
│   ├── InsOdo.java         # 里程计辅助
│   ├── InsMagnetometer.java # 磁力计航向辅助
│   └── InsStateIdx.java    # EKF 状态向量索引管理
├── ekf/                    # 扩展卡尔曼滤波
│   └── InsEkf.java         # EKF 初始化、预测、F 矩阵构建
└── smooth/                 # 平滑算法
    ├── InsFbs.java         # 前向-后向平滑
    └── InsRts.java         # RTS 固定区间平滑
```

### 核心算法

#### 1. 机械编排 (InsMech)

INS 机械编排是系统的核心，实现导航状态的时间传播：

- **正向编排** `updateins()`：ECEF 框架下的姿态/速度/位置更新
  - IMU 误差模型应用（零偏、标度因数、非正交）
  - 旋转和划桨补偿（sculling correction）
  - 姿态更新（四元数/DCM）
  - 速度更新（比力 + 重力 + 科氏力）
  - 位置更新
- **NED 框架编排** `updateinsn()`：导航系下的机械编排
- **IMU 调整** `adjustimu()`：坐标变换、增量/速率格式转换、角度制/弧度制转换

#### 2. 对准 (InsAlignMech)

- **粗对准** `coarseAlign()`：基于重力/地球自转的解析式粗对准
  - 静态检测 → 平均加计/陀螺 → 计算 Cbn
- **精对准** `fineAlign()`：EKF 精对准
  - 速度匹配法
  - 水平速度 + 航向角观测
- **运动中对准**：基于 GNSS 速度辅助的对准

#### 3. EKF (InsEkf)

扩展卡尔曼滤波框架：

- **状态向量**：姿态误差(3) + 速度误差(3) + 位置误差(3) + 加计零偏(3) + 陀螺零偏(3) + 可选（标度因数、非正交、杆臂、时间等）
- **初始化** `initEkf()`：根据配置确定状态维度，设置初始协方差
- **预测** `predict()`：Φ·P·Φᵀ + Q
- **F 矩阵构建** `buildF()`：9×9 基础 + 误差状态扩充
- **Q 矩阵构建** `buildQ()`：PSD 参数驱动的过程噪声

#### 4. 辅助约束 (Aiding)

| 算法 | 类 | 观测量 | 约束条件 |
|------|-----|--------|----------|
| NHC | InsNhc | 侧向/垂向速度 = 0 | 车辆非滑移 |
| ZVU | InsZvu | 速度 = 0 | 静止/零速 |
| ZARU | InsZaru | 角速率 = 0 | 静止/零转 |
| 里程计 | InsOdo | 前向速度 | 已知标度因数 |
| 磁力计 | InsMagnetometer | 航向角 | 已知参考磁场 |

#### 5. 平滑 (Smoothing)

| 算法 | 类 | 说明 |
|------|-----|------|
| FBS | InsFbs | 前向-后向平滑，两次滤波结果加权平均 |
| RTS | InsRts | Rauch-Tung-Striebel 固定区间平滑，最优估计 |

### InsProviderImpl — 契约实现

`InsProviderImpl` 是 INS 模块对 `InsProvider` 接口的完整实现，负责：

1. **配置映射**：将 `InsConfig` 映射为内部 `InsOpt` 参数
2. **IMU 数据转换**：`ImuMeasurement` → `Imud`
3. **时间更新流程**：IMU 预处理 → 机械编排 → EKF 预测 → 辅助约束
4. **预测输出**：提取位置/速度/姿态及协方差子块
5. **状态修正**：应用融合层的 dx/dP 修正
6. **健康监测**：基于协方差迹和 INS 状态自动判断

---

## ignav-gnss — GNSS 卫导模块

### 包结构

```
org.gnss.ignav.gnss/
└── GnssProviderImpl.java   # GnssProvider 契约完整实现
```

### 核心功能

`GnssProviderImpl` 是 GNSS 模块对 `GnssProvider` 接口的完整实现，基于 rtklib-java 库提供 GNSS 定位解算能力。

#### 1. SPP 单点定位

- 调用 `PntPos.pntpos()` 实现标准单点定位
- 支持多系统（GPS/GLONASS/Galileo/BDS）
- 高度角截止、GDOP 计算

#### 2. RTK 相对定位

- 调用 `RtkCore.rtkpos()` 实现实时动态定位
- TC 模式下优先尝试 RTK，失败回退 SPP
- 非 TC 模式直接使用 SPP

#### 3. 紧组合观测构建

`computeObservation()` 为紧组合模式构建 EKF 观测方程：

```
伪距新息:  v = P_obs - P_pred - CLIGHT·dts
观测矩阵:  H = [-e, 0, 0, ...]  (方向矢量)
多普勒新息: v = -f·(ṙ·e - v_ins·e) / CLIGHT
观测矩阵:  H = [0, -e, 0, ...]  (速度方向矢量)
噪声矩阵:  R = σ²/sin²(el)      (高度角加权)
```

- 基于 INS 预测位置计算卫星几何
- 卫星位置通过 `EphModel.satpos()` 计算
- 新息门限检验（`maxPositionInnovation`）

#### 4. RTCM 数据流处理

- 内置 `RtcmCallbackDecoder` 实时解码 RTCM 流
- 自动接收星历（Eph/Geph）和 SSR 改正
- 观测历元回调

#### 5. RINEX 文件加载

- `loadRinexObs()`：加载 RINEX 观测文件
- `loadRinexNav()`：加载 RINEX 导航文件
- 通过 `RinexParser` 解析

#### 6. INS 预测辅助

- `setInsPrediction()` 接收 INS 预测
- 时间同步检查（`maxSyncTimeDiff`）
- 辅助周跳探测和模糊度固定

### 配置映射

| GnssConfig 参数 | PrcOpt 映射 |
|-----------------|-------------|
| `fusionMode` | 决定 SPP/RTK 选择策略 |
| `posMeasurementNoise` | 伪距观测噪声 |
| `velMeasurementNoise` | 多普勒观测噪声 |
| `minSatellitesForTc` | TC 模式最低卫星数 |

### 健康监测

```java
健康状态判断:
- NOMINAL:    有解 + 卫星数 ≥ 6 + GNSS 数据新鲜
- GNSS_DEGRADED: 有解 + 卫星数 < 6 或 GDOP > 10
- GNSS_ONLY:  INS 未提供
- FAILED:     无解或 GNSS 数据超时（> 30s）
```

---

## ignav-fusion — 融合层

### 包结构

```
org.gnss.ignav.fusion/
├── IgnavFusion.java        # 融合主编排器
├── EkfFusion.java          # EKF 融合引擎
├── FusionConfig.java       # 融合层配置
└── FusionTimeProvider.java # 时间同步实现
```

### IgnavFusion — 融合主编排器

`IgnavFusion` 是整个系统的核心协调器，管理 INS/GNSS 双向数据流和融合模式切换。

#### 初始化流程

```
init(InsConfig, GnssConfig)
  ├→ insProvider.configure(insConfig)
  ├→ gnssProvider.configure(gnssConfig)
  ├→ insProvider.setTimeProvider(timeProvider)
  ├→ gnssProvider.setTimeProvider(timeProvider)
  └→ ekf.init(stateDim, initState, initCov)
```

#### IMU 处理流程

```
processImu(ImuMeasurement)
  ├→ timeProvider.feedImuTime(imu.time)
  ├→ insProvider.timeUpdate(imu)
  ├→ lastInsSolution = insProvider.getSolution()
  └→ gnssProvider.setInsPrediction(insProvider.getPrediction())
```

#### GNSS 处理流程

```
processGnss()
  ├→ timeProvider.feedGnssTime(gnssTime)
  ├→ gnssProvider.solvePosition()
  ├→ 自适应模式切换 checkAdaptiveMode()
  ├→ 根据模式执行 EKF 更新:
  │   ├→ LC:  lcUpdate()   — 位置/速度观测
  │   ├→ STC: stcUpdate()  — 位置/速度 + 部分观测
  │   └→ TC:  tcUpdate()   — 原始伪距/多普勒观测
  ├→ 反馈校正 (可选): insProvider.applyCorrection(dx, dP)
  ├→ 更新 lastFusedSolution
  └→ 保存前向解 (平滑启用时): saveForwardSolution()
```

#### 平滑处理

当 `FusionConfig.enableSmoothing = true` 时，融合层自动收集前向滤波解。数据处理完成后可调用：

- **RTS 平滑** `applyRtsSmoothing()`：基于前向解执行 Rauch-Tung-Striebel 固定区间平滑，对位置/速度/姿态分别进行 3×3 协方差子块的 RTS 递推
- **FBS 平滑** `applyFbsSmoothing(backwardSolutions)`：前向-后向平滑，需要提供反向滤波结果，通过协方差逆加权融合前向和后向估计

```
RTS 平滑流程:
  1. 前向解收集: 每个 GNSS 历元保存 InsSolution 快照
  2. 反向递推: 从最后一个历元开始，计算平滑增益 Ck = Pk * Pk_pred^(-1)
  3. 状态更新: x_smooth(k) = x(k) + Ck * (x_smooth(k+1) - x(k))
  4. 协方差更新: P_smooth(k) = P(k) + Ck * (P_smooth(k+1) - P_pred(k+1)) * Ck^T

FBS 平滑流程:
  1. 前向解 + 后向解按时间对齐
  2. 协方差逆加权: P_smooth = (P_fwd^(-1) + P_bwd^(-1))^(-1)
  3. 状态加权: x_smooth = P_smooth * (P_fwd^(-1)*x_fwd + P_bwd^(-1)*x_bwd)
```

#### 自适应模式切换

```
卫星数/GDOP 判断:
  sats < 2              → INS_ONLY
  sats ≥ 2, GDOP < 20   → LC
  sats ≥ 4, GDOP < 6    → STC
  sats ≥ 6, GDOP < 3    → TC

保护机制:
  - 滞回计数: 在当前模式保持 epochHold 个历元后才允许切换
  - 冷却期: 切换后 cooldown 个历元内不再切换
  - GNSS 数据龄期检查: 超龄自动降级
```

#### INS 协方差重置

当 INS 协方差迹超过阈值（`maxInsCovForReset`）时：
1. 从 GNSS 解重新初始化 INS 状态
2. 重置 EKF 状态和协方差
3. 日志记录重置事件

### EkfFusion — EKF 融合引擎

独立的扩展卡尔曼滤波器，支持多种观测更新模式。

#### 状态向量

```
x = [δφ(3), δv(3), δr(3), ∇(3), ε(3)]ᵀ   (15 维)
     姿态   速度   位置  加计零偏 陀螺零偏
```

可通过 `stateDimension` 扩展至 18/21/24 维（含标度因数、非正交等）。

#### 预测

```
P ← Φ·P·Φᵀ + Q
```

- Φ: 状态转移矩阵（由 INS EKF 提供）
- Q: 过程噪声矩阵（由 INS EKF 提供）

#### LC 松组合更新

```
观测: z = [pos_gnss - pos_ins, vel_gnss - vel_ins]ᵀ
H = [0₃ₓ₃  0₃ₓ₃  I₃ₓ₃  0₃ₓ₃  0₃ₓ₃]    (位置)
    [0₃ₓ₃  I₃ₓ₃  0₃ₓ₃  0₃ₓ₃  0₃ₓ₃]    (速度)

S = H·P·Hᵀ + R
K = P·Hᵀ·S⁻¹
x ← x + K·(z - H·x)
P ← (I - K·H)·P·(I - K·H)ᵀ + K·R·Kᵀ   (Joseph 形式)
```

- S 矩阵求逆使用 `CommonOps_DDRM.invert()`
- 新息检验: `vᵀ·S⁻¹·v < chi2Threshold × nm`

#### TC 紧组合更新

```
观测: 来自 GnssProvider.computeObservation()
  v: 新息向量 (伪距 + 多普勒)
  H: 观测矩阵
  R: 噪声协方差 (对角阵)

更新公式同 LC，但维度由卫星数决定
```

#### STC 半紧组合更新

```
先执行 LC 位置/速度更新
再追加 TC 部分观测更新（可选）
```

#### 卡方检验

每次更新后计算新息比 `vᵀ·S⁻¹·v / nm`，超过阈值则拒绝本次更新，防止粗差污染。

### FusionConfig — 融合配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `insConfig` | 默认 InsConfig | INS 配置 |
| `gnssConfig` | 默认 GnssConfig | GNSS 配置 |
| `fusionMode` | LC | 初始融合模式 |
| `maxInsCovForReset` | 1e8 | INS 协方差重置阈值 |
| `maxGnssAgeForLc` | 30.0s | LC 模式最大 GNSS 数据龄期 |
| `maxGnssAgeForStc` | 10.0s | STC 模式最大 GNSS 数据龄期 |
| `maxGnssAgeForTc` | 5.0s | TC 模式最大 GNSS 数据龄期 |
| `enableAdaptiveMode` | true | 启用自适应模式切换 |
| `enableFeedbackCorrection` | true | 启用反馈校正 |
| `enableSmoothing` | false | 启用平滑（收集前向解，供 RTS/FBS 平滑使用） |
| `chi2Threshold` | 0.01 | 卡方检验阈值 |
| `stateDimension` | 15 | EKF 状态维度 |

### FusionTimeProvider — 时间同步

实现 `TimeProvider` 接口，负责 IMU/GNSS 时间同步。

#### 时间同步策略

```
1. IMU 时间驱动系统时间（高频 100Hz+）
2. GNSS 时间校准系统时间（低频 1Hz）
3. 当 GNSS 时间有效时，计算 IMU-GNSS 时间偏差
4. 一阶模型: bias = t_gnss - t_imu（指数平滑）
5. 二阶模型: 在一阶基础上追踪钟漂率 drift_rate = d(bias)/dt
6. 时钟漂移跟踪: 通过连续观测估计漂移率
```

#### 二阶模型

启用二阶模型后，时间同步不仅估计时间偏差，还估计钟漂率：

```
bias(k) = bias(k-1) * (1 - α) + measurement * α    // 一阶偏差滤波
drift_rate(k) = drift_rate(k-1) * (1 - β) + inst_rate * β  // 钟漂率滤波
imuToSystem(t) = t + bias + drift_rate * dt          // 补偿钟漂率项
```

其中 `α = 0.05`（偏差平滑系数），`β = 0.01`（漂移率平滑系数）。

通过 `setSecondOrderModel(true)` 启用，`getTimeDriftRate()` 获取当前钟漂率估计。

#### 关键方法

| 方法 | 说明 |
|------|------|
| `feedImuTime(GTime)` | 输入 IMU 时间戳，更新系统时间和漂移 |
| `feedGnssTime(GTime)` | 输入 GNSS 时间戳，校准系统时间 |
| `imuToSystem(GTime)` | IMU 时间 → 系统时间（补偿偏差 + 钟漂率） |
| `gnssToSystem(GTime)` | GNSS 时间 → 系统时间（补偿偏差） |
| `setTimeBias(double)` | 设置时间偏差（融合层估计） |
| `getTimeBias()` | 获取当前时间偏差 |
| `getImuTimeDrift()` | IMU 时钟漂移率 |
| `getGnssTimeDrift()` | GNSS 时钟漂移率 |
| `setSecondOrderModel(boolean)` | 启用/禁用二阶钟漂率模型 |
| `isSecondOrderModel()` | 查询二阶模型状态 |
| `getTimeDriftRate()` | 获取钟漂率估计 |
| `setTimeDriftRate(double)` | 设置钟漂率 |

---

## ignav-app — 应用层

### 包结构

```
org.gnss.ignav.app/
└── IgnavApp.java   # 应用入口
```

### IgnavApp — 应用入口

`IgnavApp` 是系统的顶层入口，封装了初始化、数据输入和处理流程。

#### 运行模式

| 模式 | 说明 |
|------|------|
| 离线处理 | 从文件加载 IMU/GNSS 数据，顺序处理 |
| 实时输入 | 通过 `feedImu()` / `feedGnss()` 接口逐历元输入 |
| Demo 模式 | 无参数运行时自动生成模拟数据演示 |

#### 离线处理流程

```
runOffline(imuFile, gnssObsFile, gnssNavFile)
  ├→ loadImuFile()           # 解析 IMU 文件 (week tow gyro_x gyro_y gyro_z accl_x accl_y accl_z)
  ├→ loadGnssObs()           # 加载 GNSS 观测文件 (RINEX)
  ├→ loadGnssNav()           # 加载 GNSS 导航文件 (RINEX)
  └→ 循环处理:
      ├→ feedImu(imu)        # 每 IMU 历元
      └→ feedGnss()          # 每 5 个 IMU 历元一次
```

#### Demo 模式

无命令行参数时自动运行 Demo：
- 生成 1000 个模拟 IMU 历元（dt=0.01s，含正弦陀螺 + 静态加速度计）
- 每 5 个 IMU 历元触发一次 GNSS 更新
- 初始位置设为 ECEF 坐标

#### 默认配置工厂

| 方法 | 说明 |
|------|------|
| `createDefaultInsConfig()` | 15 维 EKF，估计 bg/ba，启用 ZVU |
| `createDefaultGnssConfig()` | LC 模式，posNoise=2.5m，velNoise=0.1m/s |
| `createDefaultFusionConfig()` | 自适应模式，反馈校正，15 维状态 |

#### 命令行用法

```bash
# 离线处理
java -jar ignav-app.jar <imu_file> <gnss_obs_file> <gnss_nav_file>

# Demo 模式
java -jar ignav-app.jar
```

#### IMU 文件格式

```
# 注释行以 # 或 % 开头
week tow gyro_x gyro_y gyro_z accl_x accl_y accl_z
2300 432000.000 0.0001 -0.00005 0.00003 0.01 0.02 9.81
2300 432000.010 0.0002 -0.00004 0.00002 0.01 0.02 9.80
```

#### SolutionLogger

内部日志类，每 10 个 GNSS 历元输出一次融合结果：
```
SOL: t=2300/432000.000 pos=[-2674691.0000,3745950.0000,4499760.0000] vel=[0.000000,0.000000,0.000000] mode=LC sats=8 health=NOMINAL
```

---

## 数据流

### 正常导航流程

```
IMU 数据 → InsProvider.timeUpdate()
              ├→ InsMech.adjustimu()      # IMU 预处理
              ├→ InsMech.updateins()      # 机械编排
              ├→ InsEkf.predict()         # EKF 预测
              ├→ InsNhc/Zvu/Zaru/Odo     # 辅助约束
              └→ 更新 InsState

InsProvider.getPrediction() → IgnavFusion → GnssProvider.setInsPrediction()

GNSS 数据 → GnssProvider.solvePosition() / computeObservation()
              ↓
IgnavFusion → EkfFusion.update() → StateCorrection(dx, dP)
              ↓
InsProvider.applyCorrection()  (反馈校正，可选)
```

### 融合模式数据流

```
LC (松组合):  GNSS 位置/速度 ←→ INS 位置/速度
STC (半紧组合): GNSS 位置/速度 + 部分观测 ←→ INS
TC (紧组合):  GNSS 原始观测(伪距/载波) ←→ INS 预测位置
INS_ONLY:    纯 INS 递推，无外部修正
```

### 完整系统数据流

```
                    ┌─────────────────────────────────────────┐
                    │              IgnavFusion                │
                    │                                         │
  IMU ──→ processImu() ──→ InsProvider ──→ getPrediction()  │
                               │              │               │
                          timeUpdate()   setInsPrediction()   │
                               │              │               │
                               ▼              ▼               │
                          InsSolution    GnssProvider         │
                                           │     │            │
                                     solvePos() computeObs() │
                                           │     │            │
                                           ▼     ▼            │
  GNSS ──→ processGnss() ──→ EkfFusion.update()              │
                                    │                         │
                          applyCorrection() ←── StateCorrection
                                    │                         │
                                    ▼                         │
                              FusedSolution                   │
                    └─────────────────────────────────────────┘
```

---

## WGS84 常量

| 常量 | 值 | 说明 |
|------|-----|------|
| RE_WGS84 | 6378137.0 m | 长半轴 |
| FE_WGS84 | 1/298.257223563 | 扁率 |
| OMGE | 7.2921151467E-5 rad/s | 地球自转角速率 |
| MU | 3.986004418E14 m³/s² | 引力常数 |
| CLIGHT | 299792458.0 m/s | 光速 |

---

## 构建配置

### 父 POM 依赖管理

```xml
<dependencyManagement>
    EJML 0.43.1, SLF4J 2.0.9, Logback 1.4.8, JUnit 5.9.2
    + 内部模块: ignav-contract, ignav-ins, ignav-gnss, ignav-fusion, ignav-app
</dependencyManagement>
```

### 模块依赖

| 模块 | 依赖 |
|------|------|
| ignav-contract | 无（纯接口 + 数据类） |
| ignav-ins | ignav-contract, ejml-all, slf4j-api |
| ignav-gnss | ignav-contract, rtklib-java, ejml-all, slf4j-api |
| ignav-fusion | ignav-contract, ignav-ins, ignav-gnss, ejml-all, slf4j-api |
| ignav-app | ignav-fusion, logback-classic |

---

## 待实现

- [ ] 单元测试（各模块核心算法测试）

### 已实现（本次新增）

- [x] 前向-后向平滑在融合层的集成 — `IgnavFusion.applyRtsSmoothing()` / `IgnavFusion.applyFbsSmoothing()`，基于 `InsSolution` 实现独立于 `ignav-ins` 模块的 RTS/FBS 平滑，通过 `FusionConfig.enableSmoothing` 启用后自动收集前向解，处理完成后调用平滑方法
- [x] 时间同步二阶模型 — `FusionTimeProvider` 新增钟漂率估计，通过 `setSecondOrderModel(true)` 启用，启用后在时间偏差估计中追踪漂移率变化，`imuToSystem()` 补偿钟漂率项

### 不计划实现

- 杆臂补偿在线估计 — 当前杆臂为固定配置，C++ 参考代码 `Fusing.cc` 中 `removeIGArmLever()` 也仅实现杆臂补偿，**未实现在线估计**，杆臂仍为固定配置 `ig_lever`。如需在线估计，需扩展状态向量（第18-20维）并设计相应观测方程，当前无参考实现

### 已移除项（非算法库职责）

- ~~PPP 精密单点定位~~ — rtklib-java 已实现 PPP 解算，`GnssPositionSolution.SolutionStatus.PPP` 状态码已定义，`GnssProviderImpl` 中 `SOLQ_PPP` 映射已就绪
- ~~实时流式接口（NTRIP、串口）~~ — 属于应用层功能，不在算法库范围内
- ~~配置文件加载（YAML/JSON）~~ — 属于应用层功能，不在算法库范围内
- ~~结果文件输出（NMEA、KML、自定义格式）~~ — 属于应用层功能，不在算法库范围内

#### 配置文件加载 — 为什么不需要在库中实现

C++ 参考代码的 `Config.cc` 是一个 ~500 行的 YAML 解析器，功能是将 YAML 配置文件映射到 C++ 结构体。它解析的内容包括：

| 配置分组 | 内容 | Java 库中的对应 |
|---------|------|----------------|
| `inputopt_t` | 输入文件路径（RINEX、SP3、IMU、POS 等） | ❌ 库不负责文件管理 |
| `commonopt_t` | 处理模式、起止时间、日期 | ❌ 库不负责任务编排 |
| `imuopt_t` | INS 对齐方式、坐标系、辅助开关（NHC/ZVU/dop_aid/tdcp_aid） | ✅ 已在 `InsConfig` 中通过 Java 属性设置 |
| `gnssopt_t` | GNSS 采样率、模式、频点、仰角、AR 参数、天线参数 | ✅ 已在 `GnssConfig` 中通过 Java 属性设置 |
| `outputopt_t` | 输出格式、时间系统、NMEA 间隔、小数位数 | ❌ 库不负责输出格式化 |
| `rtklibopt_t` | RTKLIB 处理选项 | ✅ 由 rtklib-java 自身管理 |

**结论**：C++ 的 `Config.cc` 本质是一个"YAML → 结构体"的映射工具，**不包含任何算法逻辑**。Java 库已通过 `InsConfig`/`GnssConfig`/`FusionConfig` 提供了等价的配置能力（构造函数默认值 + setter），调用方可以自由选择用 YAML/JSON/Properties 等任意方式填充这些配置对象。配置文件加载应由上层应用负责，库只需暴露配置接口。

#### 结果文件输出 — 为什么不需要在库中实现

C++ 参考代码的结果输出功能包括：

| 函数 | 功能 | 性质 |
|------|------|------|
| `outsolhead()` | 输出结果文件头（列名、单位等） | 格式化输出 |
| `outsol()` | 输出单历元结果（LLH/ECEF/ENU 坐标 + 状态） | 格式化输出 |
| `outsolex()` | 输出扩展结果（NMEA GGA/RMC 格式） | 格式化输出 |
| `convkml()` | 转换为 Google Earth KML 格式 | 可视化输出 |
| `convgpx()` | 转换为 GPX 格式 | 可视化输出 |

**结论**：这些函数全部是**纯格式化输出**，将 `sol_t`（定位结果）转为特定文件格式，**不包含任何算法逻辑**。Java 库已通过 `InsResult`/`GnssPositionSolution` 等数据结构暴露了完整的计算结果，调用方可以自行格式化为 NMEA/KML/GPX 等任意格式。结果文件输出应由上层应用负责。

### 待实现算法在 C++ 参考代码中的实现情况

| 算法 | Java 实现状态 | C++ 参考代码是否已实现 | 参考文件 | 备注 |
|------|-------------|---------------------|---------|------|
| 前向-后向平滑集成 | ✅ 已实现 | ✅ 已实现 | `TGINS/src/fusing/Fusing.cc`, `Fbs.cc` | Java 版在 `IgnavFusion` 中基于 `InsSolution` 独立实现 RTS/FBS |
| 时间同步二阶模型 | ✅ 已实现 | ✅ 已实现 | `TGINS/src/fusing/TightCouple.cc`, `Fusing.cc` | Java 版在 `FusionTimeProvider` 中实现钟漂率估计 |
| 杆臂在线估计 | ❌ 不计划实现 | ❌ 未实现 | `TGINS/src/fusing/Fusing.cc` 仅有 `removeIGArmLever()` | C++ 版也仅实现固定杆臂补偿，无在线估计 |

**结论**：所有在 C++ 参考代码中已实现的算法，Java 版均已实现。杆臂在线估计在 C++ 中也未实现，当前不计划实现。

---

## 参考文档

- [使用文档](docs/USAGE.md) — 配置说明、默认值、典型场景
- [C++ 原版 BUG 记录](docs/BUGS_FIXED.md) — 记录 C++ 原版代码中发现并修复的 7 个 BUG
- [实现差异文档](docs/IMPLEMENTATION_DIFFERENCES.md) — Java 版与 C++ 版的实现差异对照

---

## License

Private - All Rights Reserved