from setuptools import setup
import os
from glob import glob

package_name = 'rallp'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rallp']),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.sdf'))),
        (os.path.join('share', package_name, 'meshes/DAE/base'), glob(os.path.join('meshes/DAE/base', '*.dae'))),
        (os.path.join('share', package_name, 'meshes/DAE/wheel'), glob(os.path.join('meshes/DAE/wheel', '*.dae'))),
        (os.path.join('share', package_name, 'meshes/DAE/lidar_base'), glob(os.path.join('meshes/DAE/lidar_base', '*.dae'))),
        (os.path.join('share', package_name, 'meshes/DAE/lidar_head'), glob(os.path.join('meshes/DAE/lidar_head', '*.dae'))),
        (os.path.join('share', package_name), ['app.js', 'index.html', 'styles.css', 'oorb_logo.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed-jeljli',
    maintainer_email='ahmed-jeljli@todo.todo',
    description='RallP robot package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blue_dot_control2 = rallp.blue_dot_control2:main',
        ],
    },
)