import os
from setuptools import setup
from glob import glob

package_name = 'ic120_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params', 'map_nav_params'),glob('params/map_nav_params/*.yaml')),
        (os.path.join('share', package_name, 'params', 'odom_nav_params'),glob('params/odom_nav_params/*.yaml')),
        (os.path.join('share', package_name, 'params'),glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'),glob('scripts/*.py')),
        (os.path.join('share', package_name, 'srv'),glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ws',
    maintainer_email='ws@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dump_navigation = ic120_navigation.dump_navigation:main',
            'dumpup_srv_server = ic120_navigation.dumpup_srv_server:main',
            'nav_srv_server = ic120_navigation.nav_srv_server:main',
            'poseStamped2Odometry = ic120_navigation.poseStamped2Odometry:main',
        ],
    },
)
