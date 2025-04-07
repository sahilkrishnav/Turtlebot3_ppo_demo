from setuptools import find_packages, setup

package_name = 'turtlebot3_ppo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mj',
    maintainer_email='minjae5@illinois.edu',
    description='Package for training PPO on TurtleBot3 in ROS2 Humble',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'robot_controller = turtlebot3_ppo.robot_controller:main',
        ],
    },
)
