from setuptools import setup

package_name = 'data_logger'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimm',
    maintainer_email='dongsus41@gmail.com',
    description='Data logging package for wearable robot system',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_logger_node = data_logger.temperature_logger_node:main',
        ],
    },
)
