from setuptools import find_packages, setup

package_name = 'trailer_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'simple_publisher = trailer_base.simple_publisher:main',
            'simple_subscriber = trailer_base.simple_subscriber:main',
            'simple_parameter = trailer_base.simple_parameter:main',
        ],
    },
)
