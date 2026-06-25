from setuptools import find_packages, setup

package_name = 'ffw_manual_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alex@example.com',
    description='Manual SG2 movement tools for mission testing.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sg2_lateral_jog = ffw_manual_tools.sg2_lateral_jog:main',
            'sg2_mobile_teleop = ffw_manual_tools.sg2_mobile_teleop:main',
        ],
    },
)
