from setuptools import find_packages, setup

package_name = 'delivery_robot_exploration'

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
    maintainer='harish-lalam',
    maintainer_email='harishlalam227@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'autonomous_exploration = delivery_robot_exploration.autonomous_exploration:main',
        'nav2_exploration = delivery_robot_exploration.nav2_exploration:main',
    ],

    },
)
