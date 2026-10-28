# 测试数据

本目录存放可公开分发、可复现的测试数据。**所有文件均来自公开渠道，且允许公开再分发**；
不包含任何伪造、脱敏或来源不明的数据。

## 目录结构

```
data/
└── geomag/
    ├── WMM2025.COF            地磁模型球谐系数（NOAA 官方原始格式）
    ├── WMM2025_TestValues.txt 官方参考值（100 个采样点）
    └── README-WMM-COEFS.txt   官方随包说明（来源凭证）
```

## 来源与许可

| 文件 | 来源 | 许可 |
| --- | --- | --- |
| `WMM2025.COF` | NOAA / NCEI World Magnetic Model 2025 官方发布包 `WMM2025COF.zip` | 美国联邦政府作品，公有领域（public domain） |
| `WMM2025_TestValues.txt` | 同上，随 `WMM2025COF.zip` 发布 | 同上 |
| `README-WMM-COEFS.txt` | 同上 | 同上 |

下载地址（2026-10 获取）：

- 产品页：<https://www.ncei.noaa.gov/products/world-magnetic-model>
- 系数包：<https://www.ncei.noaa.gov/sites/default/files/2024-12/WMM2025COF.zip>
- 网页版参考值：<https://www.ncei.noaa.gov/sites/default/files/2025-02/WMM2025_TEST_VALUES.txt>

文件为官方原样拷贝，未做任何改动，可用 SHA256 校验：

```
WMM2025.COF             DFA8597825AF4E0B87FF4198A5B4FB661B3C49F4CD090CD0164E0259B075582F
WMM2025_TestValues.txt  E6975B093DDDEB6153E0B23CC418425C438167E7C5B1DD795DA379CB654F5819
```

## 数据格式说明

`WMM2025.COF` 为 NOAA 旧版三字段头部格式：

```
    2025.0            WMM-2025     11/13/2024     <- 纪元年 / 模型名 / 发布日期
  1  0  -29351.8       0.0       12.0        0.0   <- n m g h dg dh
```

- `n`、`m`：球谐阶数与次数（无量纲整数，最大 12）
- `g`、`h`：主磁场高斯系数（单位 nT）
- `dg`、`dh`：长期变化（secular variation，单位 nT/yr）

该头部缺少内部格式中的阶数、有效期与高度范围，`GeoMag.readModel` 会按 WMM 规范补齐为
阶数 12、有效期 5 年（2025.0–2030.0）、高度 −1~600 km。

`WMM2025_TestValues.txt` 每行 18 个字段，前 11 个为本项目使用：

| 列 | 含义 | 单位 |
| --- | --- | --- |
| 1 | 十进制年（decimal year） | 年 |
| 2 | 椭球高（HAE） | km |
| 3 | 大地纬度 | deg |
| 4 | 大地经度 | deg |
| 5 | 磁偏角 D | deg |
| 6 | 磁倾角 I | deg |
| 7 | 水平强度 H | nT |
| 8–10 | X、Y、Z 分量 | nT |
| 11 | 总场强度 F | nT |
| 12–18 | 各量的年变化率 | deg/yr、nT/yr |

参考值角度保留 2 位小数、强度保留 3 位小数，故测试容差取 1 个最小刻度
（0.01 deg / 0.05 nT），仅用于吸收参考值自身的舍入误差。

## 使用方

- [WmmReferenceTest](../ignav-ins/src/test/java/org/gnss/ignav/ins/common/WmmReferenceTest.java)
  用这 100 个官方采样点交叉验证 `GeoMag` 的球谐计算，覆盖极点附近、赤道、南北半球与
  0–100 km 高度；实测最大偏差为 0.005 deg / 0.009 nT，处于参考值舍入精度内。

测试通过相对路径 `../data/geomag/...` 定位本目录（Surefire 以模块目录为工作目录），
并兼容在仓库根目录直接运行。

## 未随仓库分发的大型公开数据

端到端验证需要真实观测数据，这类数据体积大、且部分有再分发限制，因此**不入库**，
仅在此登记获取方式：

| 用途 | 数据集 | 获取方式 | 许可 |
| --- | --- | --- | --- |
| GNSS 观测（RINEX） | IGS 全球跟踪站观测/星历 | <https://cddis.nasa.gov/archive/gnss/data/daily/> | IGS 数据使用条款，可自由用于科研 |
| 车载 IMU + GNSS 参考轨迹 | KITTI Raw Data | <https://www.cvlibs.net/datasets/kitti/raw_data.php> | CC BY-NC-SA 3.0 |
| 无人机 IMU + 视觉 + 真值 | EuRoC MAV Dataset | <https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets> | 可自由用于科研，需引用 |
| 消费级 IMU 长时间序列 | OxIOD | <https://github.com/Oxford-PT/oxiod-dataset> | 见仓库 LICENSE |

## 新增数据的要求

1. 必须有明确、可访问的公开来源 URL；
2. 许可证允许公开再分发，并在上表登记；
3. 尽量保持原始文件不改动，改动需在此说明；
4. 记录 SHA256，便于校验与复现。