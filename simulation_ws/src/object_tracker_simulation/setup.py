try:
    from setuptools import setup, find_packages
except ImportError:
    from distutils.core import setup, find_packages

package_name = 'object_tracker_simulation'

setup(
    name=package_name,
    version='1.2.0',
    packages=find_packages(),
    data_files=[
        ('lib/' + package_name, ['scripts/run_evaluation_rl_agent.sh']),
        ('lib/' + package_name, ['scripts/run_local_rl_agent.sh']),
        ('lib/' + package_name, ['scripts/run_rollout_rl_agent.sh']),
        ('share/' + package_name + '/launch', ['launch/create_world.launch.py']),
        ('share/' + package_name + '/launch', ['launch/evaluation.launch.py']),
        ('share/' + package_name + '/launch', ['launch/local_training.launch.py']),
        ('share/' + package_name + '/launch', ['launch/distributed_training.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/empty.world']),
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='AWS RoboMaker',
    author_email='ros-contributions@amazon.com',
    maintainer='AWS RoboMaker',
    maintainer_email='ros-contributions@amazon.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'AWS RoboMaker - Trains a TurtleBot to track an object using reinforcement learning.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
