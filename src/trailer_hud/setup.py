from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'trailer_hud'

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
            "trailer_hud = trailer_hud.trailer_hud:main",
            "articulation_angle_logger = trailer_hud.articulation_angle_logger:main",
            "delay_logger = trailer_hud.delay_logger:main",
        ],
    },
)
