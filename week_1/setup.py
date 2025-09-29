from setuptools import find_packages, setup

package_name = 'week_1'

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
    maintainer='ubuntu',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description='AURO: Week 1, Practical 2, ROS2 sample nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtlebot3_drive_python = week_1.turtlebot3_drive_python:main',
            'task5_publisher = week_1.task5_publisher:main',
            'task5_graceful_termination = week_1.task5_graceful_termination:main',
            'task9_subscriber = week_1.task9_subscriber:main',
            'task14 = week_1.task14:main',
            'task16 = week_1.task16:main'
        ],
    },
)
