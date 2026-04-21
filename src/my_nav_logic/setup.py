import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'my_nav_logic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='recoomputer',
    maintainer_email='bennylin5857@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_nav = my_nav_logic.color_nav_node:main',
            'color_pub = my_nav_logic.color_publisher:main',
            'main_nav = my_nav_logic.main_nav_node:main',
            'test = my_nav_logic.test:main',
            'color_cam = my_nav_logic.color_cam:main',

        ],
    },
)
