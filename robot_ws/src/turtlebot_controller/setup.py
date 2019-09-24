# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
from setuptools import setup, find_packages

# Package meta-data.
package_name = 'turtlebot_controller'
REQUIRES_PYTHON = '>=3.5.0'

setup(
    name=package_name,
    version='0.0.1',
    package_dir={'': 'robomaker'},
    packages=find_packages(where='robomaker'),
    python_requires=REQUIRES_PYTHON,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('lib/' + package_name, ['robomaker/inference_worker.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'annoy==1.8.3',
        'boto3==1.9.23',
        'futures==3.1.1',
        'gym==0.10.5',
        'matplotlib==2.0.2',
        'netifaces==0.10.7',
        'numpy==1.13.3',
        'pandas==0.20.2',
        'Pillow==4.3.0',
        'PyYAML==4.2b1',
        'scipy==0.19.0',
        'scikit-image==0.13.0',
        'tensorflow==1.12.2',
        'rospkg==1.1.7',
    ],
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
        'Deployment package for the Object Tracker RoboMaker sample application on the Turtlebot3'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robomaker = inference_worker:main',
        ],
    },
)
