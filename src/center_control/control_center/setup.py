from setuptools import find_packages, setup

package_name = 'control_center'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arthur',
    maintainer_email='arthur009587@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'master_node = control_center.master_node:main',
            'fake_chassis = control_center.fake_chassis:main',
            'fake_arm = control_center.fake_arm:main',
            'fake_camera = control_center.fake_camera:main',
        ],
    },
)
