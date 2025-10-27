import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'week_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'models/meshes'), glob(os.path.join('models/meshes', '*.stl*'))),
        (os.path.join('share', package_name, 'models/barrel'), glob(os.path.join('models/barrel', '*'))),
        (os.path.join('share', package_name, 'models/waffle_pi'), glob(os.path.join('models/waffle_pi', '*'))),
        (os.path.join('share', package_name, 'models/materials'), glob(os.path.join('models/materials', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*')))
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
            'barrel_manager = week_5.barrel_manager:main'
        ],
    },
)
