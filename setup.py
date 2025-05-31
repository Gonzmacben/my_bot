from setuptools import setup

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_sim.launch.py', 'launch/rsp.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gonz',
    maintainer_email='your_email@example.com',
    description='Your robot package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'x_test = my_bot.x_test:main',
            'lidar_setup = my_bot.lidar_setup:main',
            'motor_bridge = my_bot.motor_bridge_node:main',
        ],
    },
)