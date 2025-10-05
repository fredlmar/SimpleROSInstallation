from setuptools import setup, find_packages

package_name = 'custom_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/custom_nodes']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manfredo',
    maintainer_email='manfred.marchl@gmx.at',
    description='Custom ROS2 nodes for monitoring and status reporting',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_node = custom_nodes.custom_node:main',
        ],
    },
)
