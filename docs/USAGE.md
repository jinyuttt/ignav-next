# ignav-next 使用文档

## 快速开始

### 1. 添加依赖

ignav-next 是 Maven 多模块项目，在你的应用模块中添加依赖：

```xml
<dependency>
    <groupId>org.gnss.ignav</groupId>
    <artifactId>ignav-fusion</artifactId>
    <version>1.0-SNAPSHOT</version>
</dependency>
```

`ignav-fusion` 会自动传递依赖 `ignav-ins`、`ignav-gnss`、`ignav-contract`。

### 2. 基本使用流程

```java
// 1. 创建配置
FusionConfig config = new FusionConfig();
// 修改配置（见下文配置说明）
config.getFusionMode().setMode(FusionMode.Mode.LC);
config.getInsConfig().setEstimateBa(true);
config.getInsConfig().setEstimateBg(true);

// 2. 创建 GNSS 和 INS 提供者
GnssProvider gnssProvider = new GnssProviderImpl();  // ignav-gnss 模块
InsProvider insProvider = new InsProviderImpl();      // ignav-ins 模块

// 3. 创建融合引擎
IgnavFusion fusion = new IgnavFusion(gnssProvider, insProvider, config);

// 4. 初始化
fusion.init(config.getInsConfig(), config.getGnssConfig());

// 5. 数据处理循环
while (hasData()) {
    ImuMeasurement imu = readNextImu();
    fusion.processImu(imu);

    if (hasNewGnssData()) {
        fusion.processGnss();
    }

    InsSolution result = fusion.getFusedSolution();
    // 使用 result...
}
```

### 3. 核心接口

| 接口 | 模块 | 职责 |
|------|------|------|
| `InsProvider` | ignav-contract | INS 时间更新、状态预测、校正应用 |
| `GnssProvider` | ignav-contract | GNSS 定位解算、原始观测值提取 |
| `IgnavFusion` | ignav-fusion | 融合编排：模式切换、EKF 更新、结果输出 |

---

## 配置说明

ignav-next 通过三个配置类管理所有参数，均提供构造函数默认值 + setter 方法。调用方可自由选择 YAML/JSON/Properties 等任意方式填充这些配置对象。

### FusionConfig — 融合全局配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `insConfig` | InsConfig | new InsConfig() | INS 子系统配置 |
| `gnssConfig` | GnssConfig | new GnssConfig() | GNSS 子系统配置 |
| `fusionMode` | FusionMode | new FusionMode() | 融合模式及切换阈值 |
| `maxInsCovForReset` | double | 1e8 | INS 协方差过大时触发重置的阈值（m²） |
| `maxGnssAgeForLc` | double | 30.0 | LC 模式允许的最大 GNSS 数据龄期（秒） |
| `maxGnssAgeForStc` | double | 10.0 | STC 模式允许的最大 GNSS 数据龄期（秒） |
| `maxGnssAgeForTc` | double | 5.0 | TC 模式允许的最大 GNSS 数据龄期（秒） |
| `enableAdaptiveMode` | boolean | true | 是否启用自适应模式切换 |
| `enableFeedbackCorrection` | boolean | true | 是否将 EKF 校正量反馈到 INS |
| `enableSmoothing` | boolean | false | 是否启用后向平滑 |
| `chi2Threshold` | double | 0.01 | 卡方检验显著性水平 |
| `stateDimension` | int | 15 | EKF 状态向量维度（15=基础，18=含杆臂，21=含里程计） |

**典型修改**：

```java
FusionConfig config = new FusionConfig();
// 切换到紧组合模式
config.getFusionMode().setMode(FusionMode.Mode.TC);
// 扩展状态向量（含杆臂估计）
config.setStateDimension(18);
// 关闭自适应切换，固定为 LC 模式
config.setEnableAdaptiveMode(false);
```

### FusionMode — 融合模式与切换策略

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mode` | Mode | LC | 当前融合模式：`INS_ONLY`/`LC`/`STC`/`TC` |
| `minSatsTC` | int | 6 | TC 模式所需最少卫星数 |
| `minSatsSTC` | int | 4 | STC 模式所需最少卫星数 |
| `minSatsLC` | int | 2 | LC 模式所需最少卫星数 |
| `maxGdopTC` | double | 3.0 | TC 模式最大 GDOP |
| `maxGdopSTC` | double | 6.0 | STC 模式最大 GDOP |
| `maxGdopLC` | double | 20.0 | LC 模式最大 GDOP |
| `hysteresisEpochs` | int | 10 | 模式切换迟滞历元数（防止频繁切换） |
| `cooldownEpochs` | int | 30 | 模式切换冷却历元数 |

**模式切换逻辑**：

```
卫星数 >= minSatsTC 且 GDOP < maxGdopTC  →  TC
卫星数 >= minSatsSTC 且 GDOP < maxGdopSTC →  STC
卫星数 >= minSatsLC 且 GDOP < maxGdopLC   →  LC
否则                                       →  INS_ONLY
```

切换需满足迟滞条件（在当前模式持续 `hysteresisEpochs` 个历元后才允许切换），切换后 `cooldownEpochs` 个历元内不再切换。

**典型修改**：

```java
FusionMode mode = new FusionMode();
// 放宽 TC 模式条件（5颗卫星即可）
mode.setMinSatsTC(5);
// 缩短迟滞，加快模式切换
mode.setHysteresisEpochs(5);
```

### InsConfig — INS 配置

#### 初始状态

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `initPosEcef` | double[3] | {0,0,0} | 初始位置（ECEF，米） |
| `initVelEcef` | double[3] | {0,0,0} | 初始速度（ECEF，米/秒） |
| `initAttQuat` | double[4] | {0,0,0,1} | 初始姿态四元数（q0+q1q2q3） |
| `initPosStd` | double[3] | {0,0,0} | 初始位置标准差（米） |
| `initVelStd` | double[3] | {0,0,0} | 初始速度标准差（米/秒） |
| `initAttStd` | double | 0.0 | 初始姿态标准差（弧度） |

#### IMU 噪声参数（功率谱密度）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `gyroNoisePsd` | double | 0.0 | 陀螺角度随机游走 (rad/s/√Hz) |
| `acclNoisePsd` | double | 0.0 | 加计速度随机游走 (m/s²/√Hz) |
| `bgPsd` | double | 0.0 | 陀螺零偏不稳定性 (rad/s/√Hz) |
| `baPsd` | double | 0.0 | 加计零偏不稳定性 (m/s²/√Hz) |
| `dtPsd` | double | 0.0 | 时间偏差 PSD (s/√Hz) |
| `sgPsd` | double | 0.0 | 陀螺比例因子 PSD |
| `saPsd` | double | 0.0 | 加计比例因子 PSD |
| `rgPsd` | double | 0.0 | 陀螺交叉耦合 PSD |
| `raPsd` | double | 0.0 | 加计交叉耦合 PSD |
| `osPsd` | double | 0.0 | 里程计比例因子 PSD |
| `olPsd` | double | 0.0 | 里程计杆臂 PSD |
| `oaPsd` | double | 0.0 | 里程计角度 PSD |
| `clkPsd` | double | 0.0 | 钟差 PSD (m/√Hz) |
| `clkrPsd` | double | 0.0 | 钟漂 PSD (m/s/√Hz) |

#### 状态估计开关

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `estimateBa` | boolean | false | 估计加计零偏 |
| `estimateBg` | boolean | true | 估计陀螺零偏 |
| `estimateDt` | boolean | false | 估计时间偏差 |
| `estimateSg` | boolean | false | 估计陀螺比例因子 |
| `estimateSa` | boolean | false | 估计加计比例因子 |
| `estimateRg` | boolean | false | 估计陀螺交叉耦合 |
| `estimateRa` | boolean | false | 估计加计交叉耦合 |
| `estimateLever` | boolean | false | 估计杆臂 |
| `estimateOdo` | boolean | false | 估计里程计参数 |
| `estimateMagnetometer` | boolean | false | 估计磁力计参数 |

> **注意**：开启状态估计时，`stateDimension` 需相应增加。基础 15 维 + 每个估计项 3 维。

#### 辅助约束开关

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enableNhc` | boolean | false | 非完整约束（NHC），假设车辆侧向/垂向速度为零 |
| `enableZvu` | boolean | false | 零速检测（ZVU），静止时速度约束 |
| `enableZaru` | boolean | false | 零角速率更新（ZARU），静止时角速率约束 |
| `enableOdo` | boolean | false | 里程计辅助 |
| `enableMagnetometer` | boolean | false | 磁力计航向辅助 |

#### 杆臂与里程计

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `leverArm` | double[3] | {0,0,0} | IMU 到天线中心的杆臂（b系，米） |
| `odoScale` | double | 0.0 | 里程计比例因子 |
| `odoLever` | double[3] | {0,0,0} | 里程计杆臂（b系，米） |
| `odoAngle` | double | 0.0 | 里程计安装角（弧度） |

**典型修改**：

```java
InsConfig insConfig = new InsConfig();
// 战术级 IMU 噪声参数
insConfig.setGyroNoisePsd(1.0e-4);
insConfig.setAcclNoisePsd(1.0e-3);
insConfig.setBgPsd(1.0e-5);
insConfig.setBaPsd(1.0e-4);
// 开启零偏估计
insConfig.setEstimateBa(true);
insConfig.setEstimateBg(true);
// 设置初始位置和不确定度
insConfig.setInitPosEcef(new double[]{-2267749.0, 5009157.0, 3221290.0});
insConfig.setInitPosStd(new double[]{5.0, 5.0, 5.0});
// 设置杆臂
insConfig.setLeverArm(new double[]{0.5, -0.3, 1.2});
// 开启 NHC
insConfig.setEnableNhc(true);
```

### GnssConfig — GNSS 配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `fusionMode` | FusionMode | LC | GNSS 侧的融合模式（LC/TC/STC） |
| `posMeasurementNoise` | double | 2.5 | 位置观测噪声标准差（米） |
| `velMeasurementNoise` | double | 0.1 | 速度观测噪声标准差（米/秒） |
| `maxPositionInnovation` | double | 1000.0 | 位置新息阈值（米），超出则拒绝 |
| `maxVelocityInnovation` | double | 100.0 | 速度新息阈值（米/秒），超出则拒绝 |
| `maxSyncTimeDiff` | double | 1.0 | 最大时间同步偏差（秒） |
| `chi2Alpha` | double | 0.01 | 卡方检验显著性水平 |
| `maxUpdateTimeInterval` | double | 60.0 | 最大更新间隔（秒），超时则降级为 INS_ONLY |
| `minSatellitesForTc` | int | 5 | TC 模式最少卫星数 |
| `enableRaim` | boolean | false | 是否启用 RAIM 自主完好性监测 |

**典型修改**：

```java
GnssConfig gnssConfig = new GnssConfig();
// 切换到紧组合
gnssConfig.setFusionMode(GnssConfig.FusionMode.TC);
// 调整观测噪声（RTK 固定解精度更高）
gnssConfig.setPosMeasurementNoise(0.1);
gnssConfig.setVelMeasurementNoise(0.02);
// 收紧新息阈值
gnssConfig.setMaxPositionInnovation(100.0);
gnssConfig.setMaxVelocityInnovation(10.0);
```

---

## IMU 噪声参数参考值

不同等级 IMU 的典型噪声参数：

| 参数 | 战术级 | 工业级 | 消费级 | 单位 |
|------|--------|--------|--------|------|
| `gyroNoisePsd` | 1.0e-4 | 5.0e-3 | 1.0e-1 | rad/s/√Hz |
| `acclNoisePsd` | 1.0e-3 | 5.0e-2 | 1.0e0 | m/s²/√Hz |
| `bgPsd` | 1.0e-5 | 1.0e-3 | 1.0e-1 | rad/s/√Hz |
| `baPsd` | 1.0e-4 | 1.0e-2 | 1.0e0 | m/s²/√Hz |

---

## 融合模式说明

### LC（松组合）

GNSS 输出位置/速度，与 INS 预测的位置/速度求差作为 EKF 观测量。

- 优点：实现简单，GNSS 解算与融合解耦
- 缺点：卫星数少时 GNSS 无法解算，融合也中断
- 适用：卫星数 >= 2，GDOP < 20

### STC（半紧组合）

GNSS 输出位置/速度 + 可用卫星数/GDOP 信息，融合层根据卫星几何调整观测噪声。

- 优点：部分利用卫星原始信息
- 缺点：仍依赖 GNSS 先解算
- 适用：卫星数 >= 4，GDOP < 6

### TC（紧组合）

GNSS 提供原始伪距/多普勒观测值，与 INS 预测的伪距/多普勒求差作为 EKF 观测量。

- 优点：卫星数少于4颗时仍可融合，抗遮挡能力强
- 缺点：实现复杂，需要原始观测值
- 适用：卫星数 >= 5，GDOP < 3

### INS_ONLY

无 GNSS 更新，纯 INS 递推。误差随时间发散。

- 触发条件：卫星数不足或 GDOP 过大
- 典型场景：隧道、地下车库

---

## 状态向量定义

EKF 状态向量维度由 `stateDimension` 和估计开关共同决定：

| 索引 | 状态量 | 维度 | 对应开关 |
|------|--------|------|---------|
| 0-2 | 姿态误差 δφ | 3 | 始终包含 |
| 3-5 | 速度误差 δv | 3 | 始终包含 |
| 6-8 | 位置误差 δp | 3 | 始终包含 |
| 9-11 | 加计零偏 ba | 3 | `estimateBa` |
| 12-14 | 陀螺零偏 bg | 3 | `estimateBg` |
| 15-17 | 时间偏差 dt | 3 | `estimateDt` |
| 18-20 | 陀螺比例因子 sg | 3 | `estimateSg` |
| 21-23 | 加计比例因子 sa | 3 | `estimateSa` |
| 24-26 | 陀螺交叉耦合 rg | 3 | `estimateRg` |
| 27-29 | 加计交叉耦合 ra | 3 | `estimateRa` |
| 30-32 | 杆臂 lever | 3 | `estimateLever` |
| 33-35 | 里程计参数 odo | 3 | `estimateOdo` |

**常用配置**：

- **15 维**（默认）：姿态 + 速度 + 位置 + 加计零偏 + 陀螺零偏
- **18 维**：15 维 + 杆臂（`estimateLever=true`）
- **21 维**：15 维 + 里程计（`estimateOdo=true`）

---

## 辅助约束说明

### NHC（非完整约束）

假设地面车辆侧向和垂向速度为零，提供 2 个速度约束观测：

```
v_b[1] = 0  (侧向速度为零)
v_b[2] = 0  (垂向速度为零)
```

适用场景：车载导航。开启方法：`insConfig.setEnableNhc(true)`

### ZVU（零速更新）

检测到载体静止时，将速度约束为零：

```
v_e = 0  (ECEF 速度为零)
```

静止检测算法：GLRT / 均值方差 / 幅值 / ARE / 里程计辅助。开启方法：`insConfig.setEnableZvu(true)`

### ZARU（零角速率更新）

检测到载体静止时，将角速率约束为零：

```
ω_b = 0  (b系角速率为零)
```

通常与 ZVU 配合使用。开启方法：`insConfig.setEnableZaru(true)`

### 里程计辅助

利用里程计提供的速度观测约束 INS 速度：

```
v_odo = scale * v_forward  (前向速度约束)
```

需要设置 `odoScale`、`odoLever`、`odoAngle`。开启方法：`insConfig.setEnableOdo(true)`

### 磁力计辅助

利用磁力计航向观测约束 INS 姿态：

```
ψ_mag = atan2(m_b[1], m_b[0])  (磁航向约束)
```

开启方法：`insConfig.setEnableMagnetometer(true)`

---

## 常见配置场景

### 场景1：车载 RTK/INS 松组合

```java
FusionConfig config = new FusionConfig();
InsConfig ins = config.getInsConfig();
GnssConfig gnss = config.getGnssConfig();
FusionMode mode = config.getFusionMode();

// 融合模式
mode.setMode(FusionMode.Mode.LC);
config.setEnableAdaptiveMode(true);

// IMU 参数（战术级）
ins.setGyroNoisePsd(1.0e-4);
ins.setAcclNoisePsd(1.0e-3);
ins.setBgPsd(1.0e-5);
ins.setBaPsd(1.0e-4);
ins.setEstimateBa(true);
ins.setEstimateBg(true);

// 杆臂
ins.setLeverArm(new double[]{0.5, -0.3, 1.2});

// 辅助约束
ins.setEnableNhc(true);
ins.setEnableZvu(true);

// GNSS 噪声（RTK 固定解）
gnss.setPosMeasurementNoise(0.1);
gnss.setVelMeasurementNoise(0.02);
```

### 场景2：紧组合抗遮挡

```java
FusionConfig config = new FusionConfig();
InsConfig ins = config.getInsConfig();
GnssConfig gnss = config.getGnssConfig();
FusionMode mode = config.getFusionMode();

// 紧组合
mode.setMode(FusionMode.Mode.TC);
mode.setMinSatsTC(5);
config.setEnableAdaptiveMode(true);

// IMU 参数（战术级）
ins.setGyroNoisePsd(1.0e-4);
ins.setAcclNoisePsd(1.0e-3);
ins.setBgPsd(1.0e-5);
ins.setBaPsd(1.0e-4);
ins.setEstimateBa(true);
ins.setEstimateBg(true);

// GNSS 噪声（伪距/多普勒）
gnss.setFusionMode(GnssConfig.FusionMode.TC);
gnss.setPosMeasurementNoise(2.5);
gnss.setVelMeasurementNoise(0.1);
```

### 场景3：低成本 IMU + NHC

```java
FusionConfig config = new FusionConfig();
InsConfig ins = config.getInsConfig();

// 低成本 IMU 参数
ins.setGyroNoisePsd(1.0e-1);
ins.setAcclNoisePsd(1.0e0);
ins.setBgPsd(1.0e-1);
ins.setBaPsd(1.0e0);
ins.setEstimateBa(true);
ins.setEstimateBg(true);

// NHC 对低成本 IMU 尤为重要
ins.setEnableNhc(true);
ins.setEnableZvu(true);
ins.setEnableZaru(true);

// 里程计辅助
ins.setEnableOdo(true);
ins.setOdoScale(1.0);
ins.setOdoLever(new double[]{1.5, 0.0, 0.0});
```

### 场景4：离线后处理 + RTS 平滑

```java
FusionConfig config = new FusionConfig();
InsConfig ins = config.getInsConfig();

// 启用平滑
config.setEnableSmoothing(true);

// IMU 参数
ins.setGyroNoisePsd(1.0e-4);
ins.setAcclNoisePsd(1.0e-3);
ins.setBgPsd(1.0e-5);
ins.setBaPsd(1.0e-4);
ins.setEstimateBa(true);
ins.setEstimateBg(true);

IgnavFusion fusion = new IgnavFusion(gnssProvider, insProvider, config);
fusion.init(ins, gnss);

// 前向滤波处理所有数据
while (hasData()) {
    fusion.processImu(readNextImu());
    if (hasNewGnssData()) {
        fusion.processGnss();
    }
}

// 执行 RTS 平滑
List<InsSolution> smoothed = fusion.applyRtsSmoothing();

// 使用平滑结果
for (InsSolution sol : smoothed) {
    System.out.printf("pos=[%.4f,%.4f,%.4f]%n",
        sol.getPosEcef()[0], sol.getPosEcef()[1], sol.getPosEcef()[2]);
}
```

### 场景5：时间同步二阶模型

```java
FusionConfig config = new FusionConfig();

// 获取时间同步提供者
IgnavFusion fusion = new IgnavFusion(gnssProvider, insProvider, config);
FusionTimeProvider timeProvider = fusion.getTimeProvider();

// 启用二阶模型（钟漂率估计）
timeProvider.setSecondOrderModel(true);

// 初始化并处理数据
fusion.init(ins, gnss);
while (hasData()) {
    fusion.processImu(readNextImu());
    if (hasNewGnssData()) {
        fusion.processGnss();
    }
}

// 查看钟漂率估计
double driftRate = timeProvider.getTimeDriftRate();
System.out.printf("Clock drift rate: %.6e s/s%n", driftRate);
```

---

## 平滑算法说明

### RTS 平滑（Rauch-Tung-Striebel）

固定区间最优平滑器，仅需要前向滤波结果。从最后一个历元开始反向递推，利用前向协方差和平滑增益修正每个历元的状态和协方差。

- **调用方式**：`fusion.applyRtsSmoothing()`
- **前提**：`enableSmoothing = true`，且已完成前向滤波
- **返回**：平滑后的 `List<InsSolution>`
- **适用**：离线后处理

### FBS 平滑（Forward-Backward Smoothing）

前向-后向平滑，需要分别执行前向和后向滤波，然后通过协方差逆加权融合两个方向的估计。

- **调用方式**：`fusion.applyFbsSmoothing(backwardSolutions)`
- **前提**：`enableSmoothing = true`，且已提供反向滤波结果
- **返回**：平滑后的 `List<InsSolution>`
- **适用**：离线后处理（需要反向滤波数据）

---

## 时间同步说明

### 一阶模型（默认）

仅估计 IMU-GNSS 时间偏差：

```
bias(k) = bias(k-1) * 0.95 + measurement * 0.05
imuToSystem(t) = t + bias
```

### 二阶模型

在偏差估计基础上追踪钟漂率：

```
bias(k) = bias(k-1) * (1 - α) + measurement * α
drift_rate(k) = drift_rate(k-1) * (1 - β) + inst_rate * β
imuToSystem(t) = t + bias + drift_rate * dt
```

- **启用**：`timeProvider.setSecondOrderModel(true)`
- **参数**：α = 0.05（偏差平滑），β = 0.01（漂移率平滑）
- **适用**：IMU 和 GNSS 时钟存在持续漂移的场景