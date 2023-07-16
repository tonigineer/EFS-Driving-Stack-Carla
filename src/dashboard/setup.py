from setuptools import setup
import os
from glob import glob

package_name = 'dashboard'

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
    maintainer='toni.lubiniecki@efs-techhub.com',
    maintainer_email='toni.lubiniecki@efs-techhub.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard = dashboard.dashboard:main'
        ],
    },
    package_dir={'': '.'}
)
