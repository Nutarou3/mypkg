from setuptools import setup
import os
from glob import glob

package_name = 'mypkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ローンチファイルをインストールするための設定 (launchフォルダ内の.pyファイルを対象)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gentoku Morimoto',
    maintainer_email='s24c1123tf@s.chibakoudai.jp',
    description='ROS 2 persistence reminder package with desktop notification',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reminder_node = mypkg.reminder_node:main',
            'notifier_node = mypkg.notifier_node:main',
        ],
    },
)
