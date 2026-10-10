import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    py_modules=['markers','fkine_functions','ikine_functions'],
    zip_safe=True,
    maintainer='cristhian',
    maintainer_email='jhonquispe7164@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_forward_kinematic = robot_control.test_forward_kinematic:main',
            'test_inverse_kinematics = robot_control.test_inverse_kinematics:main',
            'test_forward_gazebo = robot_control.test_forward_gazebo:main',
            'test_inverse_gazebo = robot_control.test_inverse_gazebo:main',
            'test_dynamics = robot_control.test_dynamics:main',
            'test_forward_dynamics = robot_control.test_forward_dynamics:main',

        ],
    },
)
