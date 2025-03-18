from setuptools import find_packages, setup

package_name = 'stepper_trajectory_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/motors.yaml']),
        ('share/' + package_name + '/launch', ['launch/dev.launch.py']),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jesper',
    maintainer_email='jesper@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stepper_trajectory_processor = stepper_trajectory_processor.stepper_trajectory_processor_node:main',
            'stepper_test_node = stepper_trajectory_processor.stepper_test_node:main',
            'stepper_joint_states_node = stepper_trajectory_processor.stepper_joint_states_node:main'
        ],
    },
)
