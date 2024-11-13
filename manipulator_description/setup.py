from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'manipulator_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/resource/urdf/', glob('resource/urdf/*')),
        ('share/' + package_name+'/resource/meshes/visual/', glob('resource/meshes/visual/*')),
        ('share/' + package_name+'/resource/meshes/collision/', glob('resource/meshes/collision/*')),
        ('share/' + package_name+'/rviz/', glob('rviz/*')),
        ('share/' + package_name+'/resource/config/', glob('resource/config/*')),
        ('share/' + package_name+'/resource/moveit2/', glob('resource/moveit2/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='vinokurov1768@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
