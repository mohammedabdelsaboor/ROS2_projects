from setuptools import find_packages, setup

package_name = 'robot_controller'

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
    maintainer='sheka',
    maintainer_email='sheka@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joystick_controller = robot_controller.joy_control:main",
            "esp_controller = robot_controller.esp_control:main",
            "esp32_serial = robot_controller.mpu_esp:main",
            "cmd_vel_to_serial = robot_controller.arduino_serial:main",


        ],
    },
)
