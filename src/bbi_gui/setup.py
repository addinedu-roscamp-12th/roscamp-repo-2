from setuptools import setup
import os
from glob import glob

package_name = 'bbi_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # UI 파일 포함
        (os.path.join('share', package_name, 'ui'),
            glob('bbi_gui/ui/*.ui')),
        # Image 파일 포함
        (os.path.join('share', package_name, 'images'),
            glob('bbi_gui/images/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@email.com',
    description='BBI GUI for robot management',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'bbi_gui = bbi_gui.main:main',
        ],
    },
)
