import os
import glob

from setuptools import find_packages, setup

package_name = 'pinky2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*launch.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ane',
    maintainer_email='ane@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auto_parking_node = pinky2.auto_parking_node:main',
            'exit_parking_node = pinky2.exit_parking_node:main',
            'pinky2_orchestrator_node = pinky2.pinky2_orchestrator_node:main',
        ],
    },
)
