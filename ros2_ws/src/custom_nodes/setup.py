from setuptools import find_packages, setup

package_name = 'custom_nodes'

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
    maintainer='Manfredo',
    maintainer_email='manfred.marchl@gmx.at',
    description='Custom ROS2 nodes for monitoring and status reporting',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'custom_node = custom_nodes.custom_node:main',
        ],
    },
)
