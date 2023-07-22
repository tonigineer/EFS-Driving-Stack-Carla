from setuptools import setup
import os
from glob import glob

package_name = 'carla_efs_vehicle_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='toni@lubiniecki.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_control_interface = carla_efs_vehicle_control.vehicle_control_interface:main',
            'simple_mpc = carla_efs_vehicle_control.simple_mpc:main'
        ],
    },
    package_dir={'': '.'}
)
