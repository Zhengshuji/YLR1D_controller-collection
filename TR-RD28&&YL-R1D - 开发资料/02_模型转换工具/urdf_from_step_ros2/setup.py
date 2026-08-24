#!/usr/bin/env python3

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'urdf_from_step_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),          # 自动发现 src/import_asembly
    package_dir={'': 'src'},                     # 源码根目录

    # 方式一：将 scripts/ 下的脚本安装为可执行文件（放在 lib/urdf_from_step/）
    scripts=['scripts/create_urdf.py'],

    # 方式二（更推荐）：如果想把脚本作为控制台命令，可以用 entry_points
    entry_points={
        'console_scripts': [
            'urdf_from_step = import_asembly.create_urdf:main',
        ],
    },

    # 数据文件（用于安装 launch 文件、配置文件、package.xml 等）
    data_files=[
        # 1. 资源索引标记（必须，否则 ros2 run 找不到包）
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 2. 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        # 3. 安装 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 4. 可选：安装 urdf 模板或配置文件
        # (os.path.join('share', package_name, 'urdf'),
        #     glob('urdf/*.urdf')),
    ],

    install_requires=[
        'setuptools',
        'tf_transformations',
        'urdf_parser_py',
        # 'pythonocc-core',   # 建议单独装，因为依赖复杂
    ],

    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Convert STEP files to URDF for ROS2',
    license='BSD',
)