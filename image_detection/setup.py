from setuptools import find_packages, setup

package_name = 'image_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
       ('share/' + package_name + '/launch', ['launch/image_detection.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahulroy',
    maintainer_email='rahulroy2909@gmail.com',
    description='Facial recognition ROS2 node using DeepFace library',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'identify=image_detection.face_finder:main',
            'detect=image_detection.face_detect:main',
        ],
    },
)
