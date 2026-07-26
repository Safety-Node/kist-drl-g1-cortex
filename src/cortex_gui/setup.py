import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cortex_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'websockets',          # ws server to the display renderer
        # pillow + numpy are needed ONLY for camera_transport=raw (lazy-imported);
        # the default compressed transport forwards JPEG bytes with no extra deps.
    ],
    zip_safe=True,
    maintainer='박성용',
    maintainer_email='park50260@gmail.com',
    description='GUI egress: ROS TaskStatus/camera -> WebSocket for the display renderer.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_bridge_node = cortex_gui.gui_bridge_node:main',
        ],
    },
)
