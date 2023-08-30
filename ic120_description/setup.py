import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'ic120_description'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/ic120_rviz.launch.py']),
        (os.path.join('share', package_name, 'media/materials/scripts'),glob('media/materials/scripts/*.material')),
        (os.path.join('share', package_name, 'media/materials/textures'),glob('media/materials/textures/*.png')),
        (os.path.join('share', package_name, 'meshes'),glob('meshes/*.png')),
        (os.path.join('share', package_name, 'meshes'),glob('meshes/*.dae')),
        (os.path.join('share', package_name, 'meshes'),glob('meshes/*.bmp')),
        (os.path.join('share', package_name, 'rviz2'),glob('rviz2/*.rviz')),
        (os.path.join('share', package_name, 'rviz2'),glob('rviz2/*.vcg')),
        (os.path.join('share', package_name, 'urdf'),glob('urdf/*.gazebo')),
        (os.path.join('share', package_name, 'urdf'),glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'),glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf'),glob('urdf/*.sdf')),
        (os.path.join('share', package_name, 'worlds'),glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='example',
    author_email='example@example.com',
    maintainer='example',
    maintainer_email='example@example.com',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=('Package containing examples of how to use the rclpy API.'
    ),
    license='Apache License, Version 2.0',
    test_suite='pytest',
    entry_points={
        'console_scripts': [
        ],
    },
)