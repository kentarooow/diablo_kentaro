from setuptools import setup

package_name = 'diablo_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kentarooow',
    entry_points={
        'console_scripts': [
            'diablo_record = diablo_sensor.diablo_record:main',
        ],
    },
)