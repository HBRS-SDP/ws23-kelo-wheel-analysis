from setuptools import setup
from glob import glob
import os

package_name = 'ws23_kelo_wheel_analysis'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Description of your package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_diag_converter = ws23_kelo_wheel_analysis.wheel_diag_converter:main',
            'wheel_analysis = ws23_kelo_wheel_analysis.wheel_analysis:main'
        ],
    },
)