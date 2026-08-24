#!/usr/bin/env python3

# OpenCascade
from OCC.Core.gp import gp_Vec, gp_Quaternion, gp_Pnt, gp_Trsf, gp_Ax1
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Solid, TopoDS_Shell
from OCC.Extend.DataExchange import write_stl_file
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform

# ROS2
import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_from_euler  # pip install tf-transformations

# Python
import numpy as np
import os
import xml.etree.ElementTree as ET
from shutil import rmtree
import sys
import argparse  # 保留，仅用于可能的后备，但实际用ROS2参数

# Custom (需确保 import_asembly 模块可用)
from import_asembly.Asembly_import import read_step_file_asembly


# ==================== 辅助函数（基本未改动） ====================
def changePos2M(segment_location):
    return [segment_location.TranslationPart().X() / 1000,
            segment_location.TranslationPart().Y() / 1000,
            segment_location.TranslationPart().Z() / 1000]

def toEuler(segment_location):
    q = np.array([segment_location.GetRotation().X(),
                  segment_location.GetRotation().Y(),
                  segment_location.GetRotation().Z(),
                  segment_location.GetRotation().W()])
    return list(euler_from_quaternion(q))

def calculateTfToRoot(joint, joint_list):
    parent_name = joint['parent']
    global_this_tf = joint['location']
    local_tf = gp_Trsf()
    found = False
    for other_joint in joint_list:
        if other_joint['child'] == parent_name:
            global_other_tf = other_joint['location']
            local_tf = global_other_tf.Inverted().Multiplied(global_this_tf)
            found = True
            break
    if not found:  # root joint
        local_tf.SetTranslation(gp_Vec(0, 0, 0))
        local_tf.SetRotation(gp_Quaternion(0, 0, 0, 1))
    return local_tf

def findOneVersionOfString(string_word, versions):
    for version in versions:
        ans = string_word.find(version)
        if ans != -1:
            return ans
    return -1

def separateRobotPartsFromStep(parts_data):
    print("Searching trough step...")
    print("Prepared " + str(len(parts_data)) + " parts data")

    joints_id_names = ["joint_", "JOINT_", "Joint_"]
    connection_word_id_names = ["_to_", "_TO_", "_To_"]
    urdf_id_names = ["urdf", "URDF", "Urdf"]

    avalibel_joint_types = ["fixed", "revolute", "prismatic"]
    joint_types_id_names = {}
    joint_types_id_names["fixed"] = ["fixed", "FIXED", "Fixed"]
    joint_types_id_names["revolute"] = ["revolute", "REVOLUTE", "Revolute"]
    joint_types_id_names["prismatic"] = ["prismatic", "PRISMATIC", "Prismatic"]

    robot_joints = []
    robot_parts = []
    robot_links = []
    root_link_name = None

    for part in parts_data:
        part_data = parts_data[part]
        if len(part_data) == 4:
            segment_name, segment_color, segment_hierarchy, segment_trans = part_data
            segment_name = segment_name.replace(" ", "-")
            segment_location = part.Location().Transformation()
            segment_position = changePos2M(segment_location)
            segment_q_orientation = [segment_location.GetRotation().X(),
                                     segment_location.GetRotation().Y(),
                                     segment_location.GetRotation().Z(),
                                     segment_location.GetRotation().W()]

            if len(segment_hierarchy) > -1:
                urdf_detected = False
                h_i = 0
                for hiarchie_names in segment_hierarchy:
                    if findOneVersionOfString(hiarchie_names, urdf_id_names) == 0:
                        urdf_detected = True
                        break
                    h_i += 1
                if urdf_detected:
                    joint_name = segment_hierarchy[h_i + 1]
                    if findOneVersionOfString(joint_name, joints_id_names) == 0:
                        connection_name = joint_name[6:]
                        connection_id_string = "_to_"
                        ind = findOneVersionOfString(connection_name, connection_word_id_names)
                        if ind == -1:  # base joint
                            ind = connection_name.find("_")
                            parent_name = connection_name[0:ind]
                            root_link_name = parent_name
                            child_name = parent_name
                            parent_name = ""
                        else:
                            parent_name = connection_name[0:ind]
                            connection_name = connection_name[len(parent_name) + len(connection_id_string):]
                            ind = connection_name.find("_")
                            child_name = connection_name[0:ind]

                        joint_data = {}
                        joint_data["name"] = parent_name + "_" + child_name
                        for test_type in avalibel_joint_types:
                            if findOneVersionOfString(segment_name, joint_types_id_names[test_type]) == 0:
                                joint_data["type"] = test_type
                                break
                        joint_data["parent"] = parent_name
                        joint_data["child"] = child_name
                        joint_data["position"] = segment_position
                        joint_data["rotation"] = segment_q_orientation
                        joint_data["location"] = segment_location
                        robot_joints.append(joint_data)

                        if child_name not in robot_links:
                            robot_links.append(child_name)
                        if parent_name != "" and parent_name not in robot_links:
                            robot_links.append(parent_name)
                        continue
                    else:
                        print("PROBLEM: Not correct naming of joints in URDF asembly")
                        print(joint_name)
                        continue

            part_for_saving = False
            if type(part) == TopoDS_Solid or type(part) == TopoDS_Shell:
                part_for_saving = True
            if part_for_saving:
                robot_part = {}
                segment_name = segment_name.replace("/", "_")
                robot_part["name"] = segment_name
                robot_part["location"] = segment_location
                robot_part["hierarchy"] = segment_hierarchy
                robot_part["part"] = part
                robot_part["color"] = [segment_color.Red(), segment_color.Green(), segment_color.Blue()]
                robot_parts.append(robot_part)
        else:
            segment_name, segment_color = parts_data[part]

    return robot_parts, robot_joints, robot_links, root_link_name

def createSTLs(robot_parts, meshes_path, mode="binary"):
    print("Preparing meshes...")
    stl_output_dir = meshes_path
    output_files = []
    file_names = []
    test_count = 0
    max_parts = 2000

    for part in robot_parts:
        if test_count > max_parts:
            print("to many parts, set higher limit!")
            break
        test_count += 1

        made_name = part["name"]
        name_counter = 0
        file_name = made_name + str(name_counter)
        while file_name in file_names:
            name_counter += 1
            file_name = made_name + str(name_counter)
        file_names.append(file_name)
        file_name = file_name + ".stl"

        output_file = os.path.join(stl_output_dir, file_name)
        trfs = gp_Trsf()
        trfs.SetScale(gp_Pnt(), 0.001)
        scaled_part = BRepBuilderAPI_Transform(part['part'], trfs).Shape()
        print("output file: " + output_file)
        write_stl_file(scaled_part, output_file, mode=mode)
        output_files.append(file_name)

    return output_files

def createMaterialsAndColors(robot_parts, robot_links, meshes_paths, root_link_name):
    print("Creating materials...")
    colors_values = []
    colors_names = []
    color_counter = 0
    materials = []

    link_meshes = {}
    for name in robot_links:
        link_meshes[name] = []

    mesh_i = 0
    for part in robot_parts:
        if part["color"] in colors_values:
            color_name = colors_names[colors_values.index(part["color"])]
        else:
            colors_values.append(part["color"])
            color_name = "color" + str(color_counter)
            colors_names.append(color_name)
            # 不生成实际的 Material 对象，只在 URDF 生成时直接构造
            color_counter += 1
        part["material_name"] = color_name

        current_name = part["name"]
        if "link_" in current_name:
            current_name = current_name[5:]
            current_name = current_name[0:current_name.find("_")]
        else:
            for parent_name in part["hierarchy"]:
                if "link_" in parent_name:
                    current_name = parent_name[5:]
                    current_name = current_name[0:current_name.find("_")]
                    break
                else:
                    current_name = root_link_name
        if current_name in robot_links:
            file_name = meshes_paths[mesh_i]
            link_meshes[current_name].append({
                "mesh_name": file_name,
                "mesh_material": color_name,
                "color_value": part["color"]
            })
        else:
            print("error: no link name: " + current_name)
        mesh_i += 1

    return robot_parts, link_meshes

def generateURDF(robot_joints, robot_links, link_meshes, root_link_name, package_name):
    # 计算局部变换
    for joint in robot_joints:
        tf = calculateTfToRoot(joint, robot_joints)
        joint["local_tf"] = tf

    # 使用 urdf_parser_py 构造 URDF（需要安装 urdf_parser_py）
    from urdf_parser_py import urdf

    robot = urdf.URDF()
    robot.name = 'fifi'
    robot.version = '1.0'
    robot.gazebos = ['control']

    # 添加关节
    for joint in robot_joints:
        if joint["parent"] != "":
            joint_limit = urdf.JointLimit(effort=1000, lower=-1.548, upper=1.548, velocity=0.5)
            or_j = toEuler(joint["local_tf"])
            pos_j = changePos2M(joint["local_tf"])
            Pos1 = urdf.Pose(xyz=pos_j, rpy=or_j)
            new_joint = urdf.Joint(
                name=joint["name"],
                parent=joint["parent"],
                child=joint["child"],
                joint_type=joint["type"],
                axis=[1, 0, 0],
                origin=Pos1,
                limit=joint_limit,
                dynamics=None,
                safety_controller=None,
                calibration=None,
                mimic=None
            )
            robot.add_joint(new_joint)
        else:
            root_location_in_step = joint["location"]

    # 添加链接和可视化
    relative_mesh_path = "meshes/"
    stl_urdf_root = "package://" + package_name + "/"  # 但此处我们只输出 urdf 字符串，路径可后续调整

    for link_name in robot_links:
        urdf_link = urdf.Link(name=link_name, visual=None, inertial=None, collision=None)
        for mesh in link_meshes[link_name]:
            # 使用绝对路径或相对路径，此处为了通用使用相对路径，用户可自行修改
            meshpath = relative_mesh_path + mesh["mesh_name"]
            Mat1 = urdf.Material(name=mesh["mesh_material"], color=urdf.Color(mesh["color_value"] + [1]))
            if link_name == root_link_name:
                mesh_location = root_location_in_step
            else:
                for joint in robot_joints:
                    if link_name == joint["child"]:
                        mesh_location = joint["location"]
                        break
            translation = changePos2M(mesh_location.Inverted())
            or_part = toEuler(mesh_location.Inverted())
            Mesh_to_joint_pose = urdf.Pose(xyz=translation, rpy=or_part)
            Vis1 = urdf.Visual(geometry=urdf.Mesh(filename=meshpath), material=Mat1, origin=Mesh_to_joint_pose, name=None)
            urdf_link.add_aggregate('visual', Vis1)
        robot.add_link(urdf_link)

    return robot

# ==================== ROS2 节点类 ====================
class URDFCreatorNode(Node):
    def __init__(self):
        super().__init__('urdf_creator_node')
        # 声明参数
        self.declare_parameter('step_file_path', '/input_step_files/robot_arm.step')
        self.declare_parameter('output_folder_path', '/output_urdf')
        self.declare_parameter('urdf_package_name', 'my_robot')  # 仅用于 urdf 内部引用
        self.declare_parameter('mesh_subfolder', 'meshes')       # stl 存放子文件夹

        self.step_file_path = self.get_parameter('step_file_path').value
        self.output_folder_path = self.get_parameter('output_folder_path').value
        self.package_name = self.get_parameter('urdf_package_name').value
        self.mesh_subfolder = self.get_parameter('mesh_subfolder').value

        self.get_logger().info('URDF Creator Node started.')
        self.get_logger().info(f'STEP file: {self.step_file_path}')
        self.get_logger().info(f'Output folder: {self.output_folder_path}')

    def run(self):
        # 1. 读取 STEP
        self.get_logger().info('Reading STEP file...')
        try:
            parts_data = read_step_file_asembly(self.step_file_path)
        except Exception as e:
            self.get_logger().error(f'Failed to read STEP file: {e}')
            return

        # 2. 提取机器人部件
        self.get_logger().info('Separating robot parts...')
        robot_parts, robot_joints, robot_links, root_link_name = separateRobotPartsFromStep(parts_data)
        self.get_logger().info(f'Found {len(robot_links)} links, {len(robot_joints)} joints.')

        # 3. 创建输出目录
        os.makedirs(self.output_folder_path, exist_ok=True)
        meshes_path = os.path.join(self.output_folder_path, self.mesh_subfolder)
        os.makedirs(meshes_path, exist_ok=True)

        # 4. 导出 STL 网格
        self.get_logger().info('Exporting STL meshes...')
        mesh_paths = createSTLs(robot_parts, meshes_path)

        # 5. 生成材质与网格映射
        self.get_logger().info('Creating materials and colors...')
        robot_parts, link_meshes = createMaterialsAndColors(robot_parts, robot_links, mesh_paths, root_link_name)

        # 6. 生成 URDF
        self.get_logger().info('Generating URDF...')
        robot = generateURDF(robot_joints, robot_links, link_meshes, root_link_name, self.package_name)

        # 7. 保存 URDF 文件
        urdf_path = os.path.join(self.output_folder_path, self.package_name + '.urdf')
        with open(urdf_path, 'w') as f:
            f.write(robot.to_xml_string())
        self.get_logger().info(f'URDF saved to {urdf_path}')

        # 可选：生成 xacro 版本（已删除，如需可自行添加）
        self.get_logger().info('Conversion finished.')

if __name__ == '__main__':
    rclpy.init(args=args)
    node = URDFCreatorNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()