import os
import re
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from xml.dom import minidom


def _resolve_yaml_refs(content: str, config_dir: str) -> str:
    """Resolve ${links.X.Y}, ${colors.X}, ${limits.X.Y} from YAML config files."""
    configs = {}
    for name in ["links", "colors", "limits", "scale", "calibration", "dynamics"]:
        path = os.path.join(config_dir, f"{name}.yaml")
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f)
            if data is not None:
                configs[name] = data

    def _resolve(match):
        expr = match.group(1).strip()
        if re.match(r'^[a-zA-Z_]\w*$', expr):
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

    return re.sub(r'\$\{([^}]+)\}', _resolve, content)


def _add_joint_damping(xml_str: str, damping: float = 0.5) -> str:
    """Parse the URDF XML and add damping="N" to every non-fixed joint.

    A small ODE physics-engine damping provides passive stability during
    the brief window before the custom effort plugin takes over. Keep the
    value LOW (<=1.0) — excessive damping fights the controller PD loop.
    """
    doc = minidom.parseString(xml_str)

    for joint in doc.getElementsByTagName("joint"):
        jtype = joint.getAttribute("type")
        if jtype == "fixed":
            continue

        dynamics = joint.getElementsByTagName("dynamics")
        if dynamics:
            continue

        dyn = doc.createElement("dynamics")
        dyn.setAttribute("damping", str(damping))
        dyn.setAttribute("friction", "0.0")
        joint.appendChild(dyn)

    return doc.toprettyxml(indent="  ", encoding=None)


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────
    pkg_description = "ylr1d_description"
    pkg_control = "ylr1d_control"
    robot_name = "ylr1d"

    from ament_index_python.packages import get_package_share_directory
    desc_share = get_package_share_directory(pkg_description)
    ctrl_share = get_package_share_directory(pkg_control)

    base_urdf_path = os.path.join(desc_share, "urdf", "ylr1d_gazebo.urdf")
    base_xacro_path = os.path.join(desc_share, "urdf", "ylr1d_gazebo.xacro")
    config_dir = os.path.join(desc_share, "config")
    effort_plugin_xacro_path = os.path.join(ctrl_share, "urdf", "ylr1d_ros2_control.xacro")

    # ── Resolve base robot URDF ────────────────────────────
    import xacro

    if os.path.exists(base_xacro_path):
        with open(base_xacro_path) as f:
            raw = f.read()
        resolved = _resolve_yaml_refs(raw, config_dir)
        tmp_xacro = tempfile.NamedTemporaryFile(mode="w", suffix=".xacro", delete=False)
        tmp_xacro.write(resolved)
        tmp_xacro.close()
        base_doc = xacro.process_file(tmp_xacro.name, mappings={"config_path": config_dir})
        os.unlink(tmp_xacro.name)
    else:
        base_doc = minidom.parse(base_urdf_path)

    base_xml = base_doc.toxml()

    # ── Merge effort plugin and joint properties ───────────
    plugin_doc = xacro.process_file(effort_plugin_xacro_path, mappings={})
    plugin_xml = plugin_doc.toxml()

    # Strip outer <robot> wrapper from plugin fragment
    robot_tag_re = re.compile(r'<robot[^>]*>(.*)</robot>', re.DOTALL)
    m = robot_tag_re.search(plugin_xml)
    plugin_inner = m.group(1) if m else plugin_xml

    # Merge into base URDF
    combined_xml = base_xml.replace("</robot>", plugin_inner + "\n</robot>")

    # ── Inject light joint damping ─────────────────────────
    combined_xml = _add_joint_damping(combined_xml)

    # ── Write final URDF to temp file ──────────────────────
    combined_urdf = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
    combined_urdf.write(combined_xml)
    combined_urdf.close()
    combined_path = combined_urdf.name

    # ── Environment ────────────────────────────────────────
    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    model_path = os.path.join(desc_share, "meshes")
    env["GAZEBO_MODEL_PATH"] = model_path + ":" + env.get("GAZEBO_MODEL_PATH", "")

    # ── ROS 2 nodes ────────────────────────────────────────

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[combined_path],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        arguments=[combined_path],
    )

    # ── Start Gazebo ───────────────────────────────────────
    start_gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose",
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
        env=env,
    )

    # ── Spawn robot entity ─────────────────────────────────
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", robot_name, "-file", combined_path,
                   "-x", "0", "-y", "0", "-z", "0.5",
                   "-R", "0", "-P", "0", "-Y", "0"],
        output="screen",
    )
    spawn_delayed = TimerAction(period=6.0, actions=[spawn_entity])

    # ── PID controller node ─────────────────────────────────
    # (has built-in default joint list, no --params-file needed)
    pid_controller_node = Node(
        package="ylr1d_control",
        executable="ylr1d_pid_controller",
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        start_gazebo,
        spawn_delayed,
        pid_controller_node,
    ])
