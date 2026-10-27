from glob import glob

from setuptools import find_packages, setup

package_name = 'omni_workshop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.config')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Torres',
    maintainer_email='davidtorrest1097@gmail.com',
    description='Workshop: kinematic model and trajectories of the Dingo-O omnidirectional robot',
    license='Apache-2.0',
    # Registered nodes: "ros2 run omni_workshop <name>"
    entry_points={
        'console_scripts': [
            'self_controller = omni_workshop.self_controller:main',
            'plotter = omni_workshop.plotter:main',
        ],
    },
)
