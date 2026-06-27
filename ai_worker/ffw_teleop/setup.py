from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ffw_teleop'
authors_info = [
    ('Sungho Woo', 'wsh@robotis.com'),
    ('Woojin Wie', 'wwj@robotis.com'),
    ('Wonho Yun', 'ywh@robotis.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)
setup(
    name=package_name,
    version='1.3.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors,
    author_email=author_emails,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='FFW teleop ROS 2 package.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyborad_control = ffw_teleop.keyboard_control:main',
            'mobile_teleop = ffw_teleop.mobile_teleop:main',
            'wrist_depth_overlay = ffw_teleop.right_wrist_depth_overlay:main',
            'right_wrist_depth_overlay = ffw_teleop.right_wrist_depth_overlay:main',
            'zed_depth_assist = ffw_teleop.zed_depth_assist:main',
            'teleop_alignment_status = ffw_teleop.alignment_status:main',
            'teleop_bandwidth_monitor = ffw_teleop.teleop_bandwidth_monitor:main',
            'teleop_cmd_vel_mux = ffw_teleop.teleop_cmd_vel_mux:main',
            'teleop_head_trajectory_mux = ffw_teleop.teleop_head_trajectory_mux:main',
            'mission_mode_manager = ffw_teleop.mission_mode_manager:main',
            'operator_layout_manager = ffw_teleop.operator_layout_manager:main',
            'operator_image_viewer = ffw_teleop.operator_image_viewer:main',
            'operator_drive_panel = ffw_teleop.operator_drive_panel:main',
        ],
    },
)
