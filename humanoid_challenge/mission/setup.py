import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jihun',
    maintainer_email='mnjihun@snu.ac.kr',
    description='System 팀 휴머노이드 챌린지 미션 시나리오 (Mission A 상태 기계)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # --- Mission A ---
            'mission_a = mission.mission_a:main',
            'mock_manipulation_a = mission.mock_manipulation_a:main',
            'mock_navigation_a = mission.mock_navigation_a:main',
            'mock_perception_a = mission.mock_perception_a:main',
            'move_base_lateral = mission.move_base_lateral_node:main',
            # --- Mission B (박스 픽: String-nav + subprocess-manip) ---
            'mission_b = mission.mission_b:main',
            'mission_b_monitor = mission.mission_b_monitor:main',
            'mission_b_monitor_demo = mission.mission_b_monitor_demo:main',
            'mock_nav_b = mission.mock_nav_b:main',
            # --- Mission C (peg 삽입/너트, A 재사용 클론) ---
            'mission_c = mission.mission_c:main',
            # --- Mission D (recovery FSM, 실기능 미구현/mock) ---
            'mission_d = mission.mission_d:main',
            'mock_manipulation_d = mission.mock_manipulation_d:main',
            'mock_navigation_d = mission.mock_navigation_d:main',
            'mock_perception_d = mission.mock_perception_d:main',
        ],
    },
)
