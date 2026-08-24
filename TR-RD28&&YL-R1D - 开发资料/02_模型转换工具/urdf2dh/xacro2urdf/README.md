# xacro2urdf

把 `urdf/ylr1d.xacro` 配合 `config/` 下的 YAML 配置，转换为**自包含**的静态 URDF（`urdf/ylr1d.urdf`）。

生成的 URDF 所有 `${...}` 占位符都已从配置文件解析内联，不依赖任何外部参数即可加载使用。仅实现当前 ylr1d 模型所需的最小 xacro 子集。

## 依赖

- Python 3.8+
- [PyYAML](https://pyyaml.org/)（`pip install -r requirements.txt`）

## 使用

```bash
cd xacro2urdf
pip install -r requirements.txt
cd ..
python -m xacro2urdf                     # 默认生成 urdf/ylr1d.urdf
python -m xacro2urdf --output out.urdf   # 指定输出路径
```

## 转换内容

1. **宏展开**：将 `<xacro:macro name="ylr1d" params="prefix">` 展开，并应用 `<xacro:ylr1d prefix="" />` 的实参。
2. **${...} 求值**：从 `config/` 读取 `links.yaml`、`colors.yaml`、`limits.yaml`、`calibration.yaml`、`dynamics.yaml`，解析 `${prefix}`、`${links.Link_Base.mass}` 等表达式（XML 注释内容不替换）。
3. **自包含**：移除外部运行时占位符 `${controllers_yaml_path}` 所在的 `<parameters>` 元素，使 URDF 不依赖 launch 注入的 controllers.yaml。
4. **规范化**：移除 fixed 关节上非法的 `<axis>`，去掉无用的 `xmlns:xacro`，属性按字母序排列。

输出后会自动做 XML 良构校验，并检查是否还有未解析的 `${...}`。

## 校验

生成的 URDF 与原始参考 `urdf/ylr1d.urdf` 逐元素（links / joints / inertia / origin / material / sensor / ros2_control）比对一致。
