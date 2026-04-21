from setuptools import setup
import os
from glob import glob

package_name = 'pose_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('pose_navigator/config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='recoomputer',
    maintainer_email='recoomputer@todo.todo',
    description='A package for navigating to predefined poses',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_navigator = pose_navigator.pose_navigator_node:main',
        ],
    },
)
