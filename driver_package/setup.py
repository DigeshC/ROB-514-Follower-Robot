from setuptools import setup

package_name = 'driver_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsingh',
    maintainer_email='you@example.com',
    description='Moves TurtleBot3 forward until spacebar is pressed',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_until_distance = driver_package.forward_until_distance:main',
            'send_home = driver_package.send_home:main'
        ],
    },
)
