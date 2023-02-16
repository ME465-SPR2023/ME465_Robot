from setuptools import setup
from glob import glob

package_name = 'ME465_Robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name, glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cameron Devine PhD',
    maintainer_email='cdevine@stmartin.edu',
    description='Support files for the robot used in ME 465 at Saint Martin\'s University in the spring of 2023.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stats_node = ME465_Robot.stats_node:main',
            'camera_info_node = ME465_Robot.camera_info_hack:main',
        ],
    },
)
