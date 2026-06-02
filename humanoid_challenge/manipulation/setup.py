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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamin',
    maintainer_email='chlgkals0730@gmail.com',
    description='Manipulation stack for the 2026 Humanoid Challenge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'test_move_to_pose = manipulation.tests.test_move_to_pose:main',
            'move_home = manipulation.tests.move_home:main',
            'move_to_pose = manipulation.tests.move_to_pose:main',
            'demo_0513 = manipulation.tests.demo_0513:main',
            'gpd_dual_view = manipulation.tests.gpd_dual_view_node:main',
            'pc_transformer = manipulation.skill_primitives.point_cloud_transformer_node:main',
            'demo_0520 = manipulation.tests.demo_0520:main',
            'demo_0521 = manipulation.tests.demo_0521:main',
            'test_gpd_open3d = manipulation.tests.test_gpd_open3d:main',
        ],
    },
)
