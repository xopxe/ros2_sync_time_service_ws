from setuptools import find_packages, setup

package_name = 'sync_time'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jvisca',
    maintainer_email='jvisca@fing.edu.uy',
    description='Time Synchronization Service',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sync_time_node = sync_time.sync_time_node:main'
        ],
    },
)
