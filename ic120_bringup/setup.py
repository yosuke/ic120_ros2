import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'ic120_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='g3',
    maintainer_email='ws@todo.todo',
    description='The ic120_bringup package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
