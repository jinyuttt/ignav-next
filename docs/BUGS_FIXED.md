# C++ 原版 BUG 记录

本文档记录在将 IGNAV C++ 代码转换为 Java 过程中发现并修复的 C++ 原版 BUG。

---

## 1. `shval3` 中 `r` 和 `ratio` 重复计算

**源文件**: `geomag.cc` → `GeoMag.java`

**严重程度**: 🔴 严重（影响地磁场计算精度）

**问题描述**:

C++ 原版 `shval3` 函数中，`r` 和 `ratio` 被计算了两次：

```c++
// 第一次计算（正确）
if (coord_sys == GEODETIC) {
    // ... 从椭球参数计算 r ...
    ratio = earths_radius / r;
    // ... 旋转 slat, clat ...
}

// 第二次计算（错误！使用了旋转后的 slat/clat）
if (coord_sys == GEODETIC) {
    aa = a2 * clat * clat;  // clat 已被旋转修改
    bb = b2 * slat * slat;  // slat 已被旋转修改
    cc = aa + bb;
    dd = sqrt(cc);
    argument = elev * (elev + 2.0 * dd) + (a2 * aa + b2 * bb) / cc;
    r = sqrt(argument);
}
ratio = earths_radius / (r != 0 ? r : elev);
```

第二次计算使用了经过 geodetic 旋转修正后的 `slat`/`clat`，导致 `r` 和 `ratio` 的值不正确。在中纬度地区偏差较小（约0.0001°），但在极地附近偏差会显著增大。

**Java 修复**:

删除重复计算，只在 geodetic 分支内计算一次 `r`，统一在分支外计算 `ratio`：

```java
double r = elev;  // 默认值（geocentric 情况）
if (coordSys == COORDSYS_GEODETIC) {
    // ... 从椭球参数计算 r ...
    // ... 旋转 slat, clat ...
}
ratio = earthsRadius / r;  // 只计算一次
```

---

## 2. `jacob_head2att` 数组赋值错误

**源文件**: `ins_magnetometer.cc` → `InsMagnetometer.java`

**严重程度**: 🔴 严重（雅可比矩阵计算错误，影响EKF滤波）

**问题描述**:

C++ 原版 `jacob_head2att` 函数中，`dhdatt[0]` 被赋值两次，`dhdatt[2]` 未被赋值：

```c++
dhdatt[0] = (Cbn[0] * (Cne[5]*Cbe[1] - Cne[4]*Cbe[2])
           - Cbn[1] * (Cne[2]*Cbe[1] - Cne[1]*Cbe[2]))
           / (SQR(Cbn[0]) + SQR(Cbn[1]));

dhdatt[0] = (Cbn[0] * (-Cne[5]*Cbe[0] + Cne[3]*Cbe[2])   // ← 错误！应为 dhdatt[1]
           - Cbn[1] * (-Cne[2]*Cbe[1] + Cne[0]*Cbe[2]))
           / (SQR(Cbn[0]) + SQR(Cbn[1]));

// dhdatt[2] 未赋值！
```

这导致磁航向对俯仰角（pitch）的偏导数被覆盖，对横滚角（roll）的偏导数缺失。

**Java 修复**:

```java
dhdatt[0] = (Cbn[0] * (Cne[5]*Cbe[1] - Cne[4]*Cbe[2])
           - Cbn[1] * (Cne[2]*Cbe[1] - Cne[1]*Cbe[2]))
           / (InsMath.SQR(Cbn[0]) + InsMath.SQR(Cbn[1]));

dhdatt[1] = (Cbn[0] * (-Cne[5]*Cbe[0] + Cne[3]*Cbe[2])   // ← 修正为 dhdatt[1]
           - Cbn[1] * (-Cne[2]*Cbe[1] + Cne[0]*Cbe[2]))
           / (InsMath.SQR(Cbn[0]) + InsMath.SQR(Cbn[1]));

dhdatt[2] = (Cbn[0] * (Cne[4]*Cbe[0] - Cne[3]*Cbe[1])    // ← 新增 dhdatt[2]
           - Cbn[1] * (Cne[1]*Cbe[0] - Cne[0]*Cbe[1]))
           / (InsMath.SQR(Cbn[0]) + InsMath.SQR(Cbn[1]));
```

---

## 3. `undecli` 函数单位不一致

**源文件**: `ins_magnetometer.cc` → `InsMagnetometer.java`

**严重程度**: 🟡 中等（影响磁偏角补偿精度）

**问题描述**:

C++ 原版 `undecli` 函数中，`pos[0]`（纬度，弧度）直接传递给 `get_field_components`，但 `get_field_components` 内部调用 `GeoMag.getFieldComponents` 期望的纬度单位是**度**：

```c++
static void undecli(...) {
    // pos[0] 是弧度
    get_field_components(pos[0], pos[1], ...);  // ← 传弧度，但函数期望度
    ...
}
```

这导致地磁模型使用错误的纬度值计算磁偏角，纬度值被放大了约57.3倍（180/π），使得磁偏角补偿完全错误。

**Java 修复**:

添加弧度到度的转换：

```java
GeoMag.getFieldComponents(pos[0] * IgnavConstants.R2D, pos[1] * IgnavConstants.R2D, ...);
```

---

## 4. `read_model` 循环起始索引跳过第一个模型

**源文件**: `geomag.cc` → `GeoMag.java`

**严重程度**: 🟡 中等（影响IGRF模型解析）

**问题描述**:

C++ 原版 `read_model` 函数中，循环从 `modelI = 1` 开始，跳过了第0个模型：

```c++
for (modelI = 1; modelI < model->nmodel; modelI++) {
    // 从第1个模型开始，跳过第0个
}
```

对于 IGRF 模型（包含24个时期的系数），第0个模型（1900.0年）的系数被跳过，导致时间插值时缺少基准数据。

**Java 修复**:

循环从 `i = 0` 开始，读取所有模型：

```java
for (int i = 0; i < model.nmodel; i++) {
    // 从第0个模型开始，读取全部
}
```

---

## 5. `magfilt` 中 `yaw` 计算单位不一致

**源文件**: `ins_magnetometer.cc` → `InsMagnetometer.java`

**严重程度**: 🟢 低（仅影响日志输出，不影响滤波结果）

**问题描述**:

C++ 原版 `magfilt` 函数中，`yaw` 的计算混合了弧度和度：

```c++
yaw = atan2(mag[1], mag[0]) * D2R;  // atan2返回弧度，乘以D2R变成"度/弧度"的混合值
```

`atan2` 返回弧度，乘以 `D2R`（度→弧度转换因子）后得到的是一个无意义的混合单位值。正确的做法应该是 `atan2(...) * R2D`（弧度→度）或直接使用 `atan2(...)` 保持弧度。

**Java 处理**:

忠实保留C++原版行为，未修改（因为仅影响日志输出，不影响EKF滤波结果）。后续版本可考虑修正。

---

## 6. `shval3` 中变量未初始化

**源文件**: `geomag.cc` → `GeoMag.java`

**严重程度**: 🟡 中等（可能导致未定义行为）

**问题描述**:

C++ 原版 `shval3` 函数中，局部变量 `fn`、`rr`、`bb` 在某些代码路径下未初始化就被使用：

```c++
double fn, rr, bb;  // 未初始化
// ... 某些分支可能不赋值就直接使用 ...
```

C++ 中未初始化的局部变量值是不确定的（undefined behavior），可能导致计算结果随机错误。

**Java 修复**:

```java
double rr = 0.0, fm = 0.0, fn = 0.0;  // 显式初始化
```

---

## 7. COF 文件头行解析方式脆弱

**源文件**: `geomag.cc` → `GeoMag.java`

**严重程度**: 🟡 中等（影响模型文件兼容性）

**问题描述**:

C++ 原版使用固定列位置解析 COF 文件头行：

```c++
model->yrmin[modelI] = atof(&inbuff[43]);
model->max1[modelI] = atoi(&inbuff[55]);
```

不同格式的 COF 文件（WMM vs IGRF）列位置可能不同，导致解析错误。

**Java 修复**:

改为按空格分隔解析，更加健壮：

```java
String[] parts = line.trim().split("\\s+");
model.yrmin[i] = Double.parseDouble(parts[2]);
model.max1[i] = Integer.parseInt(parts[3]);
```

---

## 修复验证

所有修复均通过以下验证：
- 52 个单元测试全部通过
- WMM2015 模型：declination=-4.32°, inclination=46.61°（武汉，2020年）
- IGRF12 模型：declination=-4.66°, inclination=46.01°（武汉，2020年）
- 磁力计 EKF 更新功能正常