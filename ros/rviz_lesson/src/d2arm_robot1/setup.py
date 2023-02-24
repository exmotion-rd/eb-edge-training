import glob
import os
from setuptools import setup

package_name = 'd2arm_robot1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.rviz')), 
        (os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*.urdf')), 
        (os.path.join('share', package_name), glob.glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fukuoka',
    maintainer_email='foo.bar@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
