import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'week_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'week_4 = week_4.week_4:main',
            'sample_forward_machine = week_4.sample_forward_machine:main',
            'sample_forward_turning_machine = week_4.sample_forward_turning_machine:main',
            'sample_driving_machine = week_4.sample_driving_machine:main',
            'week_4_task3_TurtleBot3FSMC = week_4.week_4_task3_TurtleBot3FSMC:main',
            'week_4_task4_DetectorFSM = week_4.week_4_task4_DetectorFSM:main',
            'week_4_task4_TurtleBot3FSMC = week_4.week_4_task4_TurtleBot3FSMC:main',
            'week_4_task5_TurtleBot3FSMC = week_4.week_4_task5_TurtleBot3FSMC:main',
            'week_4_task6_TurtleBot3FSMC = week_4.week_4_task6_TurtleBot3FSMC:main',
            'week_4_task7_PrioritisedNode = week_4.week_4_task7_PrioritisedNode:main'
        ],
    },
)
