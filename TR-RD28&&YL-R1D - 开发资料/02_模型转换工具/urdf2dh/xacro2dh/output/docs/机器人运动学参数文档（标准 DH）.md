# 机器人运动学参数文档（标准 DH）

机器人：`ylr1d`
生成工具：`xacro2dh`
源文件：`../urdf/ylr1d.xacro`、`../config`
约定：标准 DH \(Craig\)
说明：

- 标准 DH 变换：`A_i = Rz(θ)·Tz(d)·Tx(a)·Rx(α)`，其中 `x_i` 为 `z_{i-1}` 到 `z_i` 的公垂线。

- `θ_offset` 为关节变量 `q=0` 时的关节角取值。

- 正运动学计算：
`FK = base_transform · Π A_i(q) · tool_transform`，保证与 URDF 正运动学逐点一致（经测试验证）。

- 关节类型：`revolute` / `continuous` 的 `q` 作用于 `θ`；`prismatic` 的 `q` 作用于 `d`。

---

## 链：left\_arm

- 基座链接：`Link_Body2`

- 末端链接：`Link_LeftArm7`

- 关节列表（顺序）：
`Joint_Body2_to_LeftArm1` → `Joint_LeftArm1_to_LeftArm2` → `Joint_LeftArm2_to_LeftArm3` →
`Joint_LeftArm3_to_LeftArm4` → `Joint_LeftArm4_to_LeftArm5` → `Joint_LeftArm5_to_LeftArm6` →
`Joint_LeftArm6_to_LeftArm7`

### 1\.1 base\_transform（相对于 DH 第 0 帧）

- 平移：`(2.3529e-15, 0.1750, 0.1338)`

- 旋转（四元数）：`(5.5781e-31, 0.7071, 0.7071, 9.5072e-15)`

### 1\.2 tool\_transform（DH 第 n 帧 → end\_link）

- 平移：`(0.0, 2.7756e-17, 5.5511e-17)`

- 旋转（四元数）：`(0.0, 0.0, 7.8886e-31, 1.0)`

### 1\.3 DH 参数表

---

## 链：right\_arm

- 基座链接：`Link_Body2`

- 末端链接：`Link_RightArm7`

- 关节列表：
`Joint_Body2_to_RightArm1` → `Joint_RightArm1_to_RightArm2` → `Joint_RightArm2_to_RightArm3` →
`Joint_RightArm3_to_RightArm4` → `Joint_RightArm4_to_RightArm5` → `Joint_RightArm5_to_RightArm6` →
`Joint_RightArm6_to_RightArm7`

### 2\.1 base\_transform

- 平移：`(0.0, -0.1750, 0.1338)`

- 旋转（四元数）：`(-0.7071, 0.0, 0.0, 0.7071)`

### 2\.2 tool\_transform

- 平移：`(0.0, 0.0, 0.0)`

- 旋转（四元数）：`(0.0, 0.0, 1.0, 0.0)`

### 2\.3 DH 参数表

---

## 链：body

- 基座链接：`Link_Base`

- 末端链接：`Link_Body4`

- 关节列表：
`Joint_Base_to_Body1` → `Joint_Body1_to_Body2` → `Joint_Body2_to_Body3` → `Joint_Body3_to_Body4`

### 3\.1 base\_transform

- 平移：`(-0.1130, 0.0, 0.5260)`

- 旋转（四元数）：`(0.0, 0.0, 0.0, 1.0)`

### 3\.2 tool\_transform

- 平移：`(0.0, 0.0, 0.0)`

- 旋转（四元数）：`(0.0, 0.0, 1.0, 0.0)`

### 3\.3 DH 参数表

---

## 备注

- 所有角度单位均为弧度（rad），长度单位均为米（m）。

- 表中数值已按原数据保留有效精度，实际使用时可适当截断。

- 验证：正运动学与 URDF 逐点一致，可放心用于控制与仿真。



