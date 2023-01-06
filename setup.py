import os
from glob import glob
from setuptools import setup

package_name = 'frc_2468_pivision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='abhinavperi16@gmail.com',
    description='FRC 2468 Vision Package for RPI Vision Processing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localizer = frc_2468_pivision.localizer:main',
            'detection_visualizer = frc_2468_pivision.draw_detections:main'
        ],
    },
)
