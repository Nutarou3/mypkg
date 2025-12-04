from setuptools import  setup, find_packages
import os
from glob import glob
package_name = 'mypkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob(os.path.join('launch','*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gentoku3',
    maintainer_email='nutaryau3@gmail.com',
    description='a package for practice',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = mypkg.talker:main',
            'listener = mypkg.listener:main',
        ],
    },
)
