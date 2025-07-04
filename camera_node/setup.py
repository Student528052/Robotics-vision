from setuptools import find_packages, setup

package_name = 'camera_node'

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
    maintainer='ros',
    maintainer_email='528052@student.saxion.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = camera_node.camera:main',
            'sender_node = camera_node.publisher:main',
            'receiver_node = camera_node.subscriber:main',
            'robotics_node = camera_node.node_to_pose:main'
        ],
    },
)
