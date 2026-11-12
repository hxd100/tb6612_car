from setuptools import setup

package_name = 'tb6612_car'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/car_bridge.launch.py', 'launch/bringup.launch.py',
          'launch/real_bringup.launch.py', 'launch/nav2.launch.py']),
        ('share/' + package_name + '/config',
         ['config/params.yaml', 'config/car.rviz', 'config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hxd100',
    maintainer_email='hxd100@todo.todo',
    description='ROS2 bridge for the TB6612 two-wheel car',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_bridge = tb6612_car.car_bridge:main',
            'wheel_gui = tb6612_car.gui:main',
        ],
    },
)
