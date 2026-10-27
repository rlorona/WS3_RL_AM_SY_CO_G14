from glob import glob

from setuptools import find_packages, setup

package_name = 'taller_omni'

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
    description='Taller: modelo cinematico y trayectorias del robot omnidireccional Dingo-O',
    license='Apache-2.0',
    # Nodos ya registrados: "ros2 run taller_omni <nombre>"
    entry_points={
        'console_scripts': [
            'self_controller = taller_omni.self_controller:main',
            'graficar = taller_omni.graficar:main',
        ],
    },
)
