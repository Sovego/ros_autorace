from setuptools import find_packages, setup

package_name = 'Sofronov_Egor_autorace_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pechenkanepnep',
    maintainer_email='pechenkanepnep@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line = Sofronov_Egor_autorace_core.main:main',
            'camera = Sofronov_Egor_autorace_core.camera_node:main',
        ],
    },
)
