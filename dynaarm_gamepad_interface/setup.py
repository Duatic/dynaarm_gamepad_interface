import os
from setuptools import find_packages, setup

package_name = 'dynaarm_gamepad_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'config'), ['config/gamepad_config.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Schwimo',
    maintainer_email='timo.schwarzer@gmail.com',    
    tests_require=['pytest'],
    description='Modular gamepad interface for DynaArm using ROS 2.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'gamepad_interface = dynaarm_gamepad_interface.main:main'
        ],
    },    
)
