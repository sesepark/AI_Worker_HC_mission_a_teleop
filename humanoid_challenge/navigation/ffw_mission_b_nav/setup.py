from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = 'ffw_mission_b_nav'
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
        (join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (join('share', package_name, 'config'), glob('config/*.yaml')),
        (join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors,
    author_email=author_emails,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='Mission B navigation ROS 2 package.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sg2_mission_b_system_nav = ffw_mission_b_nav.sg2_mission_b_system_nav:main',
        ],
    },
)
