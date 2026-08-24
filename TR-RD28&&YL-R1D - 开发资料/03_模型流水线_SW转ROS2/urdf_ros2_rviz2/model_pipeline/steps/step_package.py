"""Step 8: Generate a complete ROS2 package from the processed model.

Creates the package directory structure with:
- CMakeLists.txt (ament_cmake)
- package.xml
- model.config (for Gazebo model database)
- urdf/*.urdf and urdf/*.xacro
- meshes/*.STL
- config/*.yaml
- launch/*.launch.py files
- rviz/display.rviz

Standard ROS2 package layout (大型项目规范):
  pkg_name/
  ├── urdf/       # .urdf and .xacro model files
  ├── meshes/     # .STL mesh files
  ├── config/     # .yaml configuration files
  ├── launch/     # .launch.py launch files
  ├── rviz/       # .rviz RViz config files
  ├── CMakeLists.txt
  └── package.xml
"""

import logging
import os
import shutil
from pathlib import Path

logger = logging.getLogger(__name__)


def generate_ros2_package(
    source_dir: str,
    pkg_dir: str,
    model_name: str,
    has_xacro: bool = True,
    sensors_yaml: str = None,
):
    """Generate a complete ROS2 package from processed model.

    Args:
        source_dir: Source directory with processed model files
        pkg_dir: Target package directory
        model_name: Model name (used for file naming)
        has_xacro: Whether Xacro files were generated
    """
    source = Path(source_dir)
    pkg = Path(pkg_dir)

    logger.info(f"Generating ROS2 package: {pkg_dir}")

    # Create directory structure (standard ROS2 layout)
    dirs = ["urdf", "meshes", "config", "launch", "rviz"]
    for d in dirs:
        (pkg / d).mkdir(parents=True, exist_ok=True)

    # 1. Copy meshes
    meshes_src = source / "meshes"
    meshes_dst = pkg / "meshes"
    if meshes_src.exists():
        _copy_meshes(meshes_src, meshes_dst)

    # 2. Copy URDF/Xacro files and fix package:// paths
    for ext in [".urdf", ".xacro"]:
        for f in source.glob(f"*{ext}"):
            dest = pkg / "urdf" / f.name
            _copy_and_fix_paths(f, dest, model_name)

    # 3. Copy config files
    config_src = source / "config"
    config_dst = pkg / "config"
    if config_src.exists():
        _copy_config(config_src, config_dst)

    # 4. Generate package manifest files
    _generate_cmakelists(pkg, model_name)
    _generate_package_xml(pkg, model_name)
    _generate_model_config(pkg, model_name)

    # 5. Generate launch files
    _generate_launch_files(pkg, model_name, has_xacro)

    # 6. Generate RViz config
    _generate_rviz_config(pkg, model_name, sensors_yaml)

    logger.info(f"ROS2 package generated successfully at: {pkg_dir}")


def _copy_meshes(src: Path, dst: Path):
    """Copy mesh files, skipping any that already exist."""
    count = 0
    for f in src.glob("*.[Ss][Tt][Ll]"):
        dest_file = dst / f.name
        if not dest_file.exists():
            shutil.copy2(str(f), str(dest_file))
            count += 1
    logger.info(f"Copied {count} mesh file(s) to {dst}")


def _copy_and_fix_paths(src: Path, dst: Path, model_name: str):
    """Copy a URDF/Xacro file and fix package:// paths.

    Standard ROS2 layout:
      package://{pkg_name}/meshes/xxx.STL
    """
    try:
        with open(src, "r", encoding="utf-8") as f:
            content = f.read()
    except Exception as e:
        logger.warning(f"Could not read {src}: {e}")
        return

    import re
    pkg_name = f"{model_name}_description"

    # Convert relative mesh paths (meshes/xxx.STL) to package:// paths
    pkg_mesh_prefix = f"package://{pkg_name}/meshes/"
    content = re.sub(
        r'(?<=filename=")meshes/',
        pkg_mesh_prefix,
        content,
    )

    # Replace any package:// references (e.g. package://Robot/meshes/)
    content = re.sub(
        r'package://([^/]+)/meshes/',
        f'package://{pkg_name}/meshes/',
        content,
    )

    # Also handle bare package:// references
    content = re.sub(
        rf'package://{re.escape(model_name)}/',
        f'package://{pkg_name}/',
        content,
    )

    with open(dst, "w", encoding="utf-8") as f:
        f.write(content)

    logger.info(f"Copied and fixed paths: {src.name} -> {dst}")


def _copy_config(src: Path, dst: Path):
    """Copy config files.

    Skips sensors.yaml (combined file) to avoid duplication with
    the individual per-sensor files in sensors/ subdirectory.
    """
    count = 0
    for f in src.glob("*"):
        if f.is_file() and f.name != "sensors.yaml":
            shutil.copy2(str(f), str(dst / f.name))
            count += 1
    # Copy sensors/ subdirectory (per-sensor config files)
    sensors_src = src / "sensors"
    sensors_dst = dst / "sensors"
    if sensors_src.exists() and sensors_src.is_dir():
        sensors_dst.mkdir(exist_ok=True)
        for f in sensors_src.glob("*"):
            if f.is_file():
                shutil.copy2(str(f), str(sensors_dst / f.name))
                count += 1
    if count > 0:
        logger.info(f"Copied {count} config file(s)")


def _generate_cmakelists(pkg: Path, model_name: str):
    """Generate CMakeLists.txt for ROS2 ament_cmake package."""
    pkg_name = f"{model_name}_description"
    content = f"""cmake_minimum_required(VERSION 3.8)
project({pkg_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY urdf/ DESTINATION share/${{PROJECT_NAME}}/urdf)
install(DIRECTORY meshes/ DESTINATION share/${{PROJECT_NAME}}/meshes)
install(DIRECTORY config/ DESTINATION share/${{PROJECT_NAME}}/config)
install(DIRECTORY launch/ DESTINATION share/${{PROJECT_NAME}}/launch)
install(DIRECTORY rviz/ DESTINATION share/${{PROJECT_NAME}}/rviz)

ament_package()
"""
    with open(pkg / "CMakeLists.txt", "w") as f:
        f.write(content.lstrip())
    logger.info("Generated: CMakeLists.txt")


def _generate_package_xml(pkg: Path, model_name: str):
    """Generate package.xml for ROS2."""
    pkg_name = f"{model_name}_description"
    content = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypesxsd="1"?>
<package format="3">
  <name>{pkg_name}</name>
  <version>1.0.0</version>
  <description>URDF description for {model_name}</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
    with open(pkg / "package.xml", "w") as f:
        f.write(content.lstrip())
    logger.info("Generated: package.xml")


def _generate_model_config(pkg: Path, model_name: str):
    """Generate model.config for Gazebo model database."""
    content = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>user</name>
    <email>user@example.com</email>
  </author>

  <description>
    URDF description for {model_name}
  </description>
</model>
"""
    with open(pkg / "model.config", "w") as f:
        f.write(content.lstrip())
    logger.info("Generated: model.config")


def _generate_launch_files(pkg: Path, model_name: str, has_xacro: bool):
    """Generate ROS2 launch files."""
    pkg_name = f"{model_name}_description"

    # URDF display (Rviz)
    _write_urdf_display_launch(pkg, pkg_name, model_name)

    # URDF gazebo
    _write_urdf_gazebo_launch(pkg, pkg_name, model_name)

    if has_xacro:
        # Xacro display (Rviz)
        _write_xacro_display_launch(pkg, pkg_name, model_name)

        # Xacro gazebo
        _write_xacro_gazebo_launch(pkg, pkg_name, model_name)


def _write_urdf_display_launch(pkg: Path, pkg_name: str, model_name: str):
    """Generate launch file for URDF display in RViz."""
    content = f'''import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "{pkg_name}"
    default_urdf = "{model_name}.urdf"
    default_rviz = "display.rviz"

    declare_urdf = DeclareLaunchArgument(
        "urdf_name", default_value=default_urdf,
        description="URDF file name (in urdf/ directory)"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz,
        description="RViz config file name (in rviz/ directory)"
    )

    urdf_name = LaunchConfiguration("urdf_name")
    rviz_config = LaunchConfiguration("rviz_config")

    urdf_path = PathJoinSubstitution([
        FindPackageShare(package_name), "urdf", urdf_name
    ])
    rviz_path = PathJoinSubstitution([
        FindPackageShare(package_name), "rviz", rviz_config
    ])

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_path],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        arguments=[urdf_path],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        env=env,
    )

    return LaunchDescription([
        declare_urdf,
        declare_rviz,
        robot_state_publisher,
        joint_state_publisher,
        rviz2,
    ])
'''
    with open(pkg / "launch" / "urdf_display.launch.py", "w") as f:
        f.write(content.lstrip())
    logger.info("Generated: launch/urdf_display.launch.py")


def _write_urdf_gazebo_launch(pkg: Path, pkg_name: str, model_name: str):
    """Generate launch file for URDF in Gazebo."""
    content = f'''import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "{pkg_name}"
    robot_name = "{model_name}"
    urdf_name = "{model_name}_gazebo.urdf"

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GAZEBO_MODEL_DATABASE_URI"] = ""

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, "urdf", urdf_name)
    rviz_path = os.path.join(pkg_share, "rviz", "gazebo_display.rviz")

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        arguments=[urdf_path],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_path],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        env=env,
    )

    start_gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose",
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
        env=env,
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", robot_name, "-file", urdf_path],
        output="screen",
    )

    spawn_delayed = TimerAction(period=5.0, actions=[spawn_entity])

    ld = LaunchDescription()
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(start_gazebo)
    ld.add_action(spawn_delayed)
    ld.add_action(rviz2)
    return ld
'''
    with open(pkg / "launch" / "urdf_gazebo.launch.py", "w") as f:
        f.write(content.lstrip())
    logger.info("Generated: launch/urdf_gazebo.launch.py")


def _write_xacro_display_launch(pkg: Path, pkg_name: str, model_name: str):
    """Generate launch file for Xacro display in RViz."""
    content = f'''import os
import re
import tempfile
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def _resolve_yaml_refs(content: str, config_dir: str) -> str:
    """Resolve ${{links.X.Y}}, ${{colors.X}}, ${{limits.X.Y}} from YAML config files.

    Replaces dotted expressions with actual values from config YAML files.
    Simple variable names like ${{prefix}} are left untouched for xacro.
    """
    configs = {{}}
    for name in ["links", "colors", "limits", "scale", "calibration", "dynamics"]:
        path = os.path.join(config_dir, f"{{name}}.yaml")
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f)
            if data is not None:
                configs[name] = data

    def _resolve(match):
        expr = match.group(1).strip()
        # Skip simple variable names (prefix, config_path) — let xacro handle them
        if re.match(r'^[a-zA-Z_]\\w*$', expr):
            return match.group(0)
        # Resolve dotted expressions from config data
        parts = expr.split(".")
        if parts[0] in configs:
            try:
                val = configs[parts[0]]
                for p in parts[1:]:
                    val = val[p]
                return str(val)
            except (KeyError, TypeError):
                pass
        return match.group(0)  # keep as-is if unresolvable

    return re.sub(r'\\$\\{{([^}}]+)\\}}', _resolve, content)


def generate_launch_description():
    package_name = "{pkg_name}"
    xacro_name = "{model_name}.xacro"
    rviz_config = "display.rviz"

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"

    pkg_share = FindPackageShare(package=package_name).find(package_name)

    xacro_path = os.path.join(pkg_share, "urdf", xacro_name)
    rviz_path = os.path.join(pkg_share, "rviz", rviz_config)
    config_dir = os.path.join(pkg_share, "config")

    # Pre-process: resolve yaml refs (${{links.X.Y}} -> actual values)
    with open(xacro_path) as f:
        raw = f.read()
    resolved = _resolve_yaml_refs(raw, config_dir)

    # Write processed content to temp file, let xacro handle ${{prefix}}
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".xacro", delete=False)
    tmp.write(resolved)
    tmp.close()

    doc = xacro.process_file(tmp.name, mappings={{"config_path": config_dir}})
    robot_desc = doc.toxml()

    os.unlink(tmp.name)  # clean up

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{{"robot_description": robot_desc}}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{{"robot_description": robot_desc}}],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        env=env,
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz2)
    return ld
'''
    with open(pkg / "launch" / "xacro_display.launch.py", "w") as f:
        f.write(content.lstrip())
    logger.info("Generated: launch/xacro_display.launch.py")


def _write_xacro_gazebo_launch(pkg: Path, pkg_name: str, model_name: str):
    """Generate launch file for Xacro in Gazebo."""
    content = f'''import os
import re
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro


def _resolve_yaml_refs(content: str, config_dir: str) -> str:
    """Resolve ${{links.X.Y}}, ${{colors.X}}, ${{limits.X.Y}} from YAML config files."""
    configs = {{}}
    for name in ["links", "colors", "limits", "scale", "calibration", "dynamics"]:
        path = os.path.join(config_dir, f"{{name}}.yaml")
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f)
            if data is not None:
                configs[name] = data

    def _resolve(match):
        expr = match.group(1).strip()
        if re.match(r'^[a-zA-Z_]\\w*$', expr):
            return match.group(0)
        parts = expr.split(".")
        if parts[0] in configs:
            try:
                val = configs[parts[0]]
                for p in parts[1:]:
                    val = val[p]
                return str(val)
            except (KeyError, TypeError):
                pass
        return match.group(0)

    return re.sub(r'\\$\\{{([^}}]+)\\}}', _resolve, content)


def generate_launch_description():
    package_name = "{pkg_name}"
    robot_name = "{model_name}"
    xacro_file = "{model_name}_gazebo.xacro"

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GAZEBO_MODEL_DATABASE_URI"] = ""

    pkg_share = get_package_share_directory(package_name)
    model_path = os.path.join(pkg_share, "meshes")
    env["GAZEBO_MODEL_PATH"] = model_path + ":" + env.get("GAZEBO_MODEL_PATH", "")

    xacro_path = os.path.join(pkg_share, "urdf", xacro_file)
    config_dir = os.path.join(pkg_share, "config")

    # Pre-process: resolve yaml refs (${{links.X.Y}} -> actual values)
    with open(xacro_path) as f:
        raw = f.read()
    resolved = _resolve_yaml_refs(raw, config_dir)

    # Write processed content to temp file, let xacro handle ${{prefix}}
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".xacro", delete=False)
    tmp.write(resolved)
    tmp.close()

    doc = xacro.process_file(tmp.name, mappings={{"config_path": config_dir}})
    robot_desc = doc.toxml()

    os.unlink(tmp.name)  # clean up xacro temp

    # Save final URDF to temp file for robot_state_publisher and Gazebo
    urdf_tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
    urdf_tmp.write(robot_desc)
    urdf_tmp.close()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_tmp.name],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        arguments=[urdf_tmp.name],
    )

    rviz_path = os.path.join(pkg_share, "rviz", "gazebo_display.rviz")

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        env=env,
    )

    start_gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose",
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
        env=env,
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", robot_name, "-file", urdf_tmp.name],
        output="screen",
    )

    spawn_delayed = TimerAction(period=5.0, actions=[spawn_entity])

    ld = LaunchDescription()
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(start_gazebo)
    ld.add_action(spawn_delayed)
    ld.add_action(rviz2)
    return ld
'''
    with open(pkg / "launch" / "xacro_gazebo.launch.py", "w") as f:
        f.write(content.lstrip())
    logger.info("Generated: launch/xacro_gazebo.launch.py")


def _generate_rviz_config(pkg: Path, model_name: str, sensors_yaml: str = None):
    """Generate a basic RViz configuration file."""
    content = f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
  - Class: rviz_common/Time
    Name: Time

Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Plane: XY
      Reference Frame: Link_Base
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Source: Topic
      Description Topic:
        Value: /robot_description
      Value: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true

  Global Options:
    Fixed Frame: Link_Base
    Frame Rate: 30

  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select

  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3
      Focal Point: {{0, 0, 0}}
      Pitch: 0.5
      Yaw: 0.65
"""
    with open(pkg / "rviz" / "display.rviz", "w") as f:
        f.write(content.lstrip())
    logger.info("Generated: rviz/display.rviz")

    # Also generate gazebo rviz config with sensor displays
    _generate_gazebo_rviz_config(pkg, model_name, sensors_yaml)


def _generate_gazebo_rviz_config(pkg: Path, model_name: str, sensors_yaml: str = None):
    """Generate RViz config for Gazebo simulation (with sensor displays).

    Uses a hand-tuned template matching the working 2.rviz reference.
    """
    _ = sensors_yaml  # unused — static template from 2.rviz
    content = f"""Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Grid1
        - /LeftCamera_rgb1
      Splitter Ratio: 0.5
    Tree Height: 562
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: LaserScan
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: Link_Base
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: RightCamera_rgb
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /right_camera/rgb/image_raw
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: GlobalCamera_rgb
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /global_camera/rgb/image_raw
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: LeftCamera_rgb
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /left_camera/rgb/image_raw
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /radar/scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: ""
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: PointCloud2
      Position Transformer: ""
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /left_camera/depth/points
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /lf_ultrasonic/range
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: Link_Base
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: false
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.4653981924057007
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.8353981375694275
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  GlobalCamera_rgb:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  LeftCamera_rgb:
    collapsed: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002bbfc0200000008fb0000001200530065006c0065006300740069006f006e000000003b0000007e0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000000200000002000000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003b000002bb000000c700fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002bbfc0200000007fb0000001e0052006900670068007400430061006d006500720061005f007200670062010000003b000001290000002800fffffffb000000200047006c006f00620061006c00430061006d006500720061005f007200670062010000016a0000005d0000002800fffffffb0000001c004c00650066007400430061006d006500720061005f00720067006201000001cd000001290000002800fffffffb0000000a0049006d006100670065010000003b000002bb0000000000000000fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003b000002bb000000a000fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b2000000000000000000000002000004b000000086fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004b000000037fc0100000002fb0000000800540069006d00650100000000000004b00000025300fffffffb0000000800540069006d006501000000000000045000000000000000000000023f000002bb00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  RightCamera_rgb:
    collapsed: false
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: -30
  Y: -32
"""
    with open(pkg / "rviz" / "gazebo_display.rviz", "w") as f:
        f.write(content.lstrip())
    logger.info("Generated: rviz/gazebo_display.rviz")
