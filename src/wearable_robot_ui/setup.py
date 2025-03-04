from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wearable_robot_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/wearable_robot_ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='개발자',
    author_email='dev@example.com',
    description='웨어러블 로봇 시스템의 사용자 인터페이스 패키지',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
        'rqt_gui.plugin': [
            'TemperatureControlPlugin = wearable_robot_ui.temperature_control_plugin:TemperatureControlPlugin',
        ],
    },

)
