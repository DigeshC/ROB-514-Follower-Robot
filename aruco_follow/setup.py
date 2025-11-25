from setuptools import find_packages, setup

package_name = 'aruco_follow'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=Aswin Arumugam',
    maintainer_email='arumugaa@oregonstate.edu',
    description='ArUco marker detection and following node for TurtleBot3 with ROS 2 Jazzy.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # FORMAT:
            # command_to_run = python_module:function
            'aruco_detector = aruco_follow.aruco_detector:main',
        ],
    },
)
