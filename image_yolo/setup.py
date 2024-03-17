from setuptools import find_packages, setup

package_name = 'image_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/image_launch.launch.xml']),
        ('share/' + package_name + '/config', ['config/marker_detect.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahulroy',
    maintainer_email='rahulroy2909@gmail.com',
    description='Human detection ROS 2 node using YOLO v8',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo = image_yolo.ros_yolo:main',
        ],
    },
)

