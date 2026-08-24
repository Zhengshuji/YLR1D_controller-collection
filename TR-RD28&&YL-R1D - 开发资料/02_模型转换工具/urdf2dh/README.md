# urdf2dh —— 模型参数提取工作区

基于 ylr1d 的权威模型（`urdf/ylr1d.xacro` + `config/` 下各 YAML 参数），提供两类自研工具，用于生成机器人参数文档所需的 DH 运动学、动力学、传感器位姿等数据。

## 目录结构

```
urdf2dh/
├── config/            # 模型参数配置（YAML）
│   ├── links.yaml     # link 质量/惯性
│   ├── colors.yaml    # 可视化颜色
│   ├── limits.yaml    # 关节限位
│   ├── scale.yaml     # 缩放参数
│   ├── calibration.yaml
│   ├── dynamics.yaml
│   ├── sensors.yaml   # 传感器清单（id/type/mount）
│   └── controllers.yaml
├── urdf/              # 权威模型：ylr1d.urdf / ylr1d.xacro
├── meshes/            # STL 网格
├── xacro2dh/          # 自研工具 A：xacro → DH/动力学/传感器参数
│   ├── run.py         # 简易入口（默认读取 ../urdf/ylr1d.xacro + ../config）
│   ├── xacro2dh/      # 核心库（expand/urdf_model/dh/kinematics/dynamics/sensors/output）
│   ├── output/        # 生成结果：kinematics.yaml / dynamics.yaml / sensors.yaml
│   └── tests/         # pytest 测试（DH FK、动力学、传感器、手算对比）
├── xacro2urdf/        # 自研工具 B：xacro → 自包含静态 URDF
│   ├── xacro2urdf.py  # 核心实现
│   ├── __main__.py    # 入口：python -m xacro2urdf
│   └── README.md      # 用法与转换内容说明
└── ThirdParty/        # 第三方 urdf2dh 库（URDF → DH 参数，仅支持串联机器人）
    ├── urdf2dh/       # 库源码
    ├── tests/         # 单元测试
    ├── res/           # 示例 URDF（irb4600、mycobot）
    └── README.md / README_ja.md
```

## 工具 A：xacro2dh（提取 DH / 动力学 / 传感器位姿）

```bash
cd xacro2dh
pip install -r requirements.txt   # numpy, PyYAML
python run.py
```

输出（`xacro2dh/output/`）：
- `kinematics.yaml` —— 各链（left_arm / right_arm / body）的标准 DH 表
- `dynamics.yaml` —— link 质量与惯性参数
- `sensors.yaml` —— 传感器位姿（零位形下基座系平移 + 安装链信息）

`run.py` 内会做 **FK 交叉验证**（DH 结果 vs URDF 结果，随机关节角），确认提取参数正确。

## 工具 B：xacro2urdf（xacro → 静态 URDF）

```bash
cd urdf2dh
python -m xacro2urdf                 # 默认生成 urdf/ylr1d.urdf
python -m xacro2urdf --output out.urdf
```

将 `urdf/ylr1d.xacro` 配合 `config/` 下的 YAML 转换为自包含的静态 URDF，所有 `${...}` 占位符内联，不依赖外部参数即可加载。详见其自带 README。

## 与报告的关系

- `kinematics.yaml` → 附录 A.2.2 机器人运动学参数文档（标准 DH）
- `dynamics.yaml` → 附录 A.2.2 机器人动力学参数文档
- `sensors.yaml` → 附录 A.2.2 机器人传感器位姿参数文档

> 整理说明：本目录为模型参数提取工作区的完整归档，内部结构未做任何改动。
