from setuptools import find_packages, setup

package_name = 'final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/final_system.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/final_view.rviz']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sean Harrington',
    maintainer_email='Harrinse@oregonstate.edu',
    description='A Package for a Patrol Robot for the ROB499/599 class at Oregon State',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = final.control_node:main',
            'patrol_node = final.patrol_node:main',
            'detector_node = final.detector_node:main',
            'camera_node = final.camera_node:main',

        ],
    },
)
