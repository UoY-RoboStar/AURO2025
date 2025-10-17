from setuptools import find_packages, setup

package_name = 'week_4'

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
            'sample_driving_machine = week_4.sample_driving_machine:main'
        ],
    },
)
