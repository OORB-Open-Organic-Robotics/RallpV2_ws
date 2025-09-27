from setuptools import setup
import os
from glob import glob

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), 
        (os.path.join('share', package_name, 'maps'), glob('maps/*')), # <-- Ajout pour rviz
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='achref',
    maintainer_email='achref@todo.todo',
    description='Navigation package with EKF config and Gazebo launch',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
