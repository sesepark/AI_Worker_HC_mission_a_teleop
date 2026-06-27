from setuptools import find_packages, setup
from glob import glob

package_name = 'manipulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data',
            ['manipulation/data/object_lut.json']),
        ('share/' + package_name + '/config',
            glob('config/*.yaml') + glob('manipulation/config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamin',
    maintainer_email='chlgkals0730@gmail.com',
    description='Manipulation stack for the 2026 Humanoid Challenge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Mission A 실 manipulation 서버 (mock_manipulation_a drop-in 대체, T1)
            'mission_a_manipulation_server = manipulation.mission_a_manipulation_server:main',
            # Mission C 실 manipulation 서버 (peg 삽입/양팔, A 재사용 — mission-c 이식)
            'mission_c_manipulation_server = manipulation.mission_c_manipulation_server:main',
            'test_zone_a       = manipulation.tests.test_zone_a:main',
            'test_pick         = manipulation.tests.test_pick:main',

            'test_home         = manipulation.tests.test_home:main',
            'test_move_to_pose   = manipulation.tests.test_move_to_pose:main',
            'test_workspace_scan = manipulation.tests.test_workspace_scan:main',
            'test_place               = manipulation.tests.test_place:main',
            'test_move_to_capture_pose   = manipulation.tests.test_move_to_capture_pose:main',
            'test_pick_no_selector       = manipulation.tests.test_pick_no_selector:main',
            'test_pick_with_perception    = manipulation.tests.test_pick_with_perception:main',
            'test_pick_with_perception_v2 = manipulation.tests.test_pick_with_perception_v2:main',
            'test_lift                   = manipulation.tests.test_lift:main',
            'test_compute_capture_pose  = manipulation.tests.test_compute_capture_pose:main',
            'test_gripper              = manipulation.tests.test_gripper:main',
            # Mission C peg/zone-c (mission-c 이식)
            'test_zone_c = manipulation.tests.test_zone_c:main',
            # 양팔/박스 픽 (Mission B subprocess 경로: test_dual_pick/test_dual_place)
            'test_dual_pick           = manipulation.tests.test_dual_pick:main',
            'test_dual_place           = manipulation.tests.test_dual_place:main',
            'test_dual_home           = manipulation.tests.test_dual_home:main',
            'test_dual_pick_old       = manipulation.tests.test_dual_pick_old:main',
            'test_dual_pick_check     = manipulation.tests.test_dual_pick_check:main',
            'test_dual_motion_save    = manipulation.tests.test_dual_motion_save:main',
            'test_dual_pick_using_motion = manipulation.tests.test_dual_pick_using_motion:main',
            'left_arm_torque_pose     = manipulation.robot_interface.left_arm_torque_pose:main',
            'test_zone_b = manipulation.tests.test_zone_b:main',
            'test_zone_b_pick = manipulation.tests.test_zone_b_pick:main',
            'test_zone_b_pick_pose = manipulation.tests.test_zone_b_pick_pose:main',
            'test_zone_b_place = manipulation.tests.test_zone_b_place:main',
            # Mission C 양팔 검증 테스트 (mission-b 이식)
            'test_pick_c            = manipulation.tests.MissionC.test_pick_C:main',
            'test_capture_to_pick_c = manipulation.tests.MissionC.test_capture_to_pick_C:main',
            'test_place_c           = manipulation.tests.MissionC.test_place_C:main',
            'test_base_c            = manipulation.tests.MissionC.test_base_C:main',
            'test_place_slots  = manipulation.tests.test_place_slots:main',
            'test_zone_clear   = manipulation.tests.test_zone_clear:main',
        ],
    },
)
