from setuptools import setup
import os
from glob import glob

package_name = 'temperature_control_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimm',
    maintainer_email='kimm@todo.todo',
    description='웨어러블 로봇 온도 및 팬 제어를 위한 rqt 플러그인',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
        'rqt_gui.plugin': [
            'temperature_control_gui = temperature_control_gui.temperature_control_gui:TemperatureControlGUI',
        ],
    },
)
