# ignav-next

INS/GNSS 组合导航系统 — 基于接口契约的多模块 Maven 项目

## 项目概述

ignav-next 是一个模块化的 INS/GNSS 组合导航系统，核心设计原则是 **INS 和 GNSS 作为两个独立演进的模块，通过 contract 层连接，融合层做主控协调双向数据流**。

## 模块结构

```
ignav-next/
├── ignav-contract/     # 共享接口和数据类型
├── ignav-ins/          # INS 惯导模块
├── ignav-gnss/         # GNSS 卫导模块（骨架）
├── ignav-fusion/       # 融合层（骨架）
└── ignav-app/          # 应用层（骨架）
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

## 数据流

### 正常导航流程

```
IMU 数据 → InsProvider.timeUpdate()
              ├→ InsMech.adjustimu()      # IMU 预处理
              ├→ InsMech.updateins()      # 机械编排
              ├→ InsEkf.predict()         # EKF 预测
              ├→ InsNhc/Zvu/Zaru/Odo     # 辅助约束
              └→ 更新 InsState

InsProvider.getPrediction() → 融合层

GNSS 数据 → GnssProvider.solvePosition() / computeObservation()
              ↓
融合层 EKF 更新 → StateCorrection(dx, dP) → InsProvider.applyCorrection()
```

### 融合模式数据流

```
LC (松组合):  GNSS 位置/速度 ←→ INS 位置/速度
STC (半紧组合): GNSS 位置/速度 + 部分观测 ←→ INS
TC (紧组合):  GNSS 原始观测(伪距/载波) ←→ INS 预测位置
INS_ONLY:    纯 INS 递推，无外部修正
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
| ignav-gnss | ignav-contract, ejml-all, slf4j-api |
| ignav-fusion | ignav-contract, ignav-ins, ignav-gnss, slf4j-api |
| ignav-app | ignav-fusion, logback-classic |

---

## 待实现

- [ ] ignav-gnss 模块实现（GNSS 数据处理、SPP/RTK/PPP）
- [ ] ignav-fusion 模块实现（LC/STC/TC 融合引擎）
- [ ] ignav-app 模块实现（配置加载、数据流驱动、结果输出）
- [ ] GnssProviderImpl 实现
- [ ] FusionEngine 实现（含 TimeProvider 和自适应模式切换）
- [ ] 单元测试
- [ ] 数据文件 I/O（RINEX、NMEA、结果文件）

---

## License

Private - All Rights Reserved