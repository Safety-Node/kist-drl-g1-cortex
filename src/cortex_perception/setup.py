import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cortex_perception'

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
        'numpy', 'scipy',            # stt speech-band filter
        'google-cloud-speech',       # stt backend
    ],
    zip_safe=True,
    maintainer='박성용',
    maintainer_email='park50260@gmail.com',
    description='Perception layer: STT (Google) and VLM scene critic.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stt_node = cortex_perception.stt_node:main',
            'vlm_node = cortex_perception.vlm_node:main',
        ],
    },
)
