from setuptools import find_packages, setup

package_name = 'pinky1'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sekim',
    maintainer_email='cksdid0907@gmail.com',
    description='pinky1 로봇 컨트롤러',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pinky1_node = pinky1.launch.main:run_single',
            'pinky1_get_pose = pinky1.get_pose:main',
        ],
    },
)
