import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cortex_cognition'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scenarios'), glob('config/scenarios/*.json5')),
    ],
    install_requires=['setuptools', 'json5'],
    zip_safe=True,
    maintainer='박성용',
    maintainer_email='park50260@gmail.com',
    description='Cognition layer: hook-driven scenario orchestrator (TaskSrv form).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orchestrator_node = cortex_cognition.orchestrator_node:main',
        ],
    },
)
