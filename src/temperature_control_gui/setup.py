from setuptools import setup

package_name = 'temperature_control_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimm',
    maintainer_email='kimm@todo.todo',
    description='Temperature control GUI for wearable robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_control_gui_node = temperature_control_gui.temperature_control_gui_node:main',
            'simple_gui = temperature_control_gui.simple_gui:main',
        ],
    },
)
