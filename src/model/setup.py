from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lidka',
    maintainer_email='lidia.luczkiewicz@pw.edu.pl',
    description='NURT robot model',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cup_publisher=model.cup_publisher:main',
            'engine=model.engine:main',
            'nozzle_position=model.nozzle_position:main'
        ],
    },
)
