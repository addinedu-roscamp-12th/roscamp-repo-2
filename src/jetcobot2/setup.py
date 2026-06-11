from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetcobot2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/jetcobot2', ['package.xml']),
        ('share/jetcobot2/launch', ['launch/launch.py']),
        (os.path.join('share', 'jetcobot_picking', 'launch'),
        glob('launch/*.py')),
    ],
    entry_points={
        'console_scripts': [
            'qr_detector = jetcobot2.qr_detector:main',
            'rack_detector = jetcobot2.rack_detector:main',
            'cv_lib = jetcobot2.cv_lib:main',
            'action_server = jetcobot2.action_server:main',
            'cv_node = jetcobot2.cv_node:main',
            'flask_node = jetcobot2.flask_node:main',
        ],
    },
)