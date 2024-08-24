from setuptools import find_packages, setup

package_name = 'butlerbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy'],
    zip_safe=True,
    maintainer='yohan',
    maintainer_email='126179055@sastra.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    
    entry_points={
        'console_scripts': [
        'order_manager = butlerbot.order_manager:main',
        'robot_controller = butlerbot.robot_controller:main',
        'confirmation = butlerbot.confirmation:main',
        'client = butlerbot.client:main',
        ],
    },
)
