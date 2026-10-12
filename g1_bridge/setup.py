from setuptools import setup

package_name = 'g1_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/g1_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiewang',
    maintainer_email='wjjy1123@163.com',
    description='ROS 2 control and wireless-controller bridge for Unitree G1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = g1_bridge.cmd_vel_bridge:main',
            'wireless_controller = g1_bridge.wireless_controller:main',
        ],
    },
)
