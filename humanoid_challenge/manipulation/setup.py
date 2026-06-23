from setuptools import find_packages, setup

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
        ('share/' + package_name + '/config', [
            'config/zone_a.yaml',
            'config/desk.yaml',
            'config/poses.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamin',
    maintainer_email='chlgkals0730@gmail.com',
    description='Manipulation stack for the 2026 Humanoid Challenge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'test_zone_a       = manipulation.tests.test_zone_a:main',
            'test_pick         = manipulation.tests.test_pick:main',

            'test_home         = manipulation.tests.test_home:main',
            'test_move_to_pose   = manipulation.tests.test_move_to_pose:main',
            'test_workspace_scan = manipulation.tests.test_workspace_scan:main',
            'test_place               = manipulation.tests.test_place:main',
            'test_move_to_capture_pose   = manipulation.tests.test_move_to_capture_pose:main',
            'test_pick_no_selector       = manipulation.tests.test_pick_no_selector:main',
            'test_pick_with_perception   = manipulation.tests.test_pick_with_perception:main',
            'test_lift                   = manipulation.tests.test_lift:main',
            'test_dual_box               = manipulation.tests.test_dual_box:main',
            'test_compute_capture_pose  = manipulation.tests.test_compute_capture_pose:main',
            'test_gripper              = manipulation.tests.test_gripper:main',
        ],
    },
)
