from setuptools import setup
import os
from glob import glob

package_name = 'rallp_torque'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', 'LICENSE']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.json'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alpha',
    maintainer_email='mansourmohamadamine@gmail.com',
    description='Torque vectoring for RallP robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'torque_vectoring_node = rallp_torque.torque_vectoring_node:main',
            'torque_vectoring_tester = rallp_torque.torque_vectoring_tester:main',
            'torque_vectoring_visualizer = rallp_torque.torque_vectoring_visualizer:main',
        ],
    },
)