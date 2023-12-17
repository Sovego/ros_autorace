from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'bagodelnya_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'signs_images'), glob(os.path.join('signs_images', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pechenkanepnep',
    maintainer_email='egor.sofronov03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_s = bagodelnya_core.main:main',
            'camera_s = bagodelnya_core.camera_node:main',
        ],
    },
)
