from setuptools import setup

package_name = 'carla_efs_vehicle_control_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'carla_efs_vehicle_control_interface = carla_efs_vehicle_control_interface.carla_efs_vehicle_control_interface:main'
        ],
    },
)
