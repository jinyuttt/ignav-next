# IGNAV-Java 实现差异文档

本文档记录 IGNAV-Java 与原始 C++ IGNAV 项目之间的实现差异。

## 1. 语言层面差异

### 1.1 内存管理

| C++ | Java | 说明 |
|-----|------|------|
| 指针/引用 | 对象引用 | Java 无指针运算 |
| malloc/free | new/GC | Java 自动垃圾回收 |
| 栈上结构体 | 堆上对象 | Java 所有对象在堆上 |
| 数组指针 | 数组对象 | Java 数组有长度信息 |

### 1.2 数据类型

| C++ | Java | 说明 |
|-----|------|------|
| double* | double[] | 一维数组替代指针 |
| double** | double[] + 索引计算 | 行优先一维数组模拟二维 |
| struct | class | Java 类替代 C 结构体 |
| enum | enum class | Java 枚举更安全 |
| int/long | int/long | Java long 固定 64 位 |

### 1.3 函数差异

| C++ | Java | 说明 |
|-----|------|------|
| 默认参数 | 方法重载 | Java 不支持默认参数 |
| 函数指针 | 函数式接口 | Java 用接口替代 |
| 全局函数 | 静态方法 | Java 所有方法在类中 |
| 宏定义 | 常量/静态方法 | Java 无预处理器宏 |

## 2. 矩阵存储差异

### 2.1 行优先 vs 列优先

C++ IGNAV 使用**行优先**存储（C 语言默认），Java 本项目也采用**行优先**存储以保持一致：

```
A[i][j] → A[i * n + j]  (行优先)
```

这与 EJML 的列优先存储不同，因此本项目使用一维 double[] 数组自行实现矩阵运算，而非直接使用 EJML 的 DenseMatrix。

### 2.2 matmul 函数

C++ 原版 rtklib 的 matmul 使用 tr 字符串参数指定转置：
- "NN": A*B
- "TN": A^T*B
- "NT": A*B^T
- "TT": A^T*B^T

Java 版本保持相同接口签名。

## 3. 全局变量处理

C++ IGNAV 大量使用全局变量（如 insstate_t ins），Java 版本将其转为类成员变量：

| C++ 全局变量 | Java 类成员 | 所属类 |
|-------------|------------|--------|
| insstate_t ins | InsState ins | InsGnss |
| insopt_t opt | InsOpt opt | InsGnss |
| imud_t imud | Imud imud | InsGnss |
| gmeas_t gnss | Gmeas gnss | InsGnss |

## 4. 日志系统差异

| C++ | Java | 说明 |
|-----|------|------|
| trace(level, fmt, ...) | logger.info/debug/warn/error | SLF4J 替代 |
| fprintf(stderr, ...) | logger.error() | 标准错误输出 |
| printf(...) | logger.info() | 标准输出 |

日志级别映射：
- trace(2, ...) → logger.debug()
- trace(3, ...) → logger.info()
- trace(4, ...) → logger.warn()
- trace(5, ...) → logger.error()

## 5. 文件 I/O 差异

C++ IGNAV 包含完整的文件读写功能（RINEX、POS 等），Java 版本：

- **保留**：POS 文件读取（PosFileReader）
- **移除**：RINEX 读取（由 rtklib_java 提供）
- **移除**：结果文件输出（由上层应用实现）
- **移除**：配置文件解析（由上层应用实现）

## 6. 数据结构差异

### 6.1 GTime

| C++ (gtime_t) | Java (GTime) | 说明 |
|---------------|-------------|------|
| time_t time | long time | Unix 时间戳（秒） |
| double sec | double sec | 小数秒 |
| - | int week | GPS 周 |
| - | double tow | 周内秒数 |

Java 版增加了 week/tow 字段，方便与 GNSS 数据交互。

### 6.2 InsState

| C++ (insstate_t) | Java (InsState) | 说明 |
|------------------|----------------|------|
| double re[3] | double[] re | ECEF 位置 |
| double ve[3] | double[] ve | ECEF 速度 |
| double Cbe[9] | double[] Cbe | 方向余弦阵 |
| double ba[3] | double[] ba | 加计零偏 |
| double bg[3] | double[] bg | 陀螺零偏 |
| double *P | double[] P | 协方差（一维数组） |
| int stat | int stat | 状态码 |
| gtime_t time | GTime time | 时间标签 |

Java 版 P 使用一维数组替代指针，大小为 nx*nx。

### 6.3 InsOpt

| C++ (insopt_t) | Java (InsOpt) | 说明 |
|----------------|---------------|------|
| int baopt | int baopt | 加计零偏估计标志 |
| int bgopt | int bgopt | 陀螺零偏估计标志 |
| int lc | int lc | 松组合标志 |
| int tc | int tc | 紧组合标志 |
| int zvu | int zvu | ZVU 标志 |
| int nhc | int nhc | NHC 标志 |
| double hz | double hz | IMU 采样率 |
| inspsd_t psd | InsPsd psd | 功率谱密度参数 |
| insunc_t unc | InsUnc unc | 初始不确定度 |
| inszvopt_t zvopt | InsZvOpt zvopt | ZVU 选项 |

## 7. 算法实现差异

### 7.1 矩阵求逆（matinv）

| C++ | Java |
|-----|------|
| rtklib matinv() | Gauss-Jordan 消元法 |

C++ 版使用 rtklib 的 matinv 函数（LU 分解），Java 版使用增广矩阵 Gauss-Jordan 消元法，数值结果等价。

### 7.2 ecef2pos 坐标转换

| C++ | Java |
|-----|------|
| Bowring 迭代法 | 改进迭代法 |

Java 版使用 z 参数迭代法，收敛精度与 C++ 版一致（<1e-12）。

### 7.3 协方差转换（covecef）

C++ 版：E * Q * E^T（直接使用 matmul33）
Java 版：E^T * Q * E（手动转置 + matmul）

两者数学上等价，但实现方式不同。covenu 保持与 C++ 一致使用 matmul33。

### 7.4 四元数运算

C++ 版直接使用数组操作四元数，Java 版额外提供了 Quaternion 工具类封装常用操作。

## 8. 接口差异

### 8.1 新增接口（Java 独有）

| 接口 | 说明 |
|------|------|
| GnssPositionResult | 标准 GNSS 定位结果结构 |
| GnssResultAdapter | 标准结构 ↔ 内部结构互转 |
| RtklibAdapter | rtklib_java 适配器 |
| PosFileReader | POS 文件读取 |
| GnssObservationProvider | 紧组合观测数据接口 |

### 8.2 移除接口

| C++ 接口 | 说明 |
|---------|------|
| stream_t | 流处理框架 |
| moni_t | 监控接口 |
| prcopt_default() | 默认选项（改为构造函数） |
| resopen/resclose | 结果文件开关 |

## 9. 编译与构建差异

| C++ | Java |
|-----|------|
| Make/CMake | Maven |
| gcc/g++ | javac |
| .h/.cpp | .java |
| 静态链接 | JAR 包 |
| 平台相关 | 跨平台 |

## 10. 测试差异

C++ IGNAV 无单元测试框架，Java 版使用 JUnit 5 编写了 38 个测试用例：

| 测试类 | 测试数 | 覆盖范围 |
|--------|--------|---------|
| InsMathTest | 14 | 数学工具函数 |
| InsGnssStateTest | 10 | 状态索引计算 |
| InsGnssIntegrationTest | 7 | 集成初始化 |
| GnssPositionResultTest | 6 | 标准结果结构 |
| PosFileReaderTest | 1 | POS 文件读取 |

## 11. 已知限制

1. **EJML 未深度集成**：当前矩阵运算使用自实现的一维数组，未充分利用 EJML 的优化
2. **紧组合未完整验证**：紧组合需要 GNSS 观测数据，目前仅框架完成
3. **RTS 平滑需前后向数据**：需要完整的前向滤波结果才能执行后向平滑
4. **POS 文件格式**：目前支持 rtklib 标准格式，其他变体可能需要扩展
5. **线程安全**：当前实现非线程安全，多线程使用需外部同步