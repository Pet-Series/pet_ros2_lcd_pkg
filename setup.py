from setuptools import setup

package_name = 'pet_ros2_lcd_pkg'

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
    maintainer='seniorKullken',
    maintainer_email='stefan.kull@gmail.com',
    description='ROS2 LCD-display controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pet_lcd_node=pet_ros2_lcd_pkg.pet_lcd_node:main",
            "display_publish_node=pet_ros2_lcd_pkg.pet_displayPublisher_node:main",
        ],
    },
)
