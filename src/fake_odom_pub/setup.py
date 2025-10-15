from setuptools import setup

package_name = 'fake_odom_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oorb',
    maintainer_email='oorb@example.com',
    description='Fake odometry publisher for slam_toolbox',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_odom_publisher = fake_odom_pub.fake_odom_publisher:main',
        ],
    },
)

