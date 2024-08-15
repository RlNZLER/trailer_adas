from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'trailer_articulation_angle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rinzler',
    maintainer_email='amelpvarghese@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "articulation_angle_node = trailer_articulation_angle.articulation_angle_node:main",
            "point_cloud = trailer_articulation_angle.point_cloud:main",
            "point_cloud_processor = trailer_articulation_angle.point_cloud_processor:main",
            "kalman_filter = trailer_articulation_angle.kalman_filter:main"
        ],
    },
)
