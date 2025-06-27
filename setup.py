from setuptools import find_packages, setup

package_name = 'autoware_loop_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'autoware_auto_vehicle_msgs'],
    zip_safe=True,
    maintainer='hyconsoft_rnd',
    maintainer_email='jh.jung@hyconsoft.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autoware_loop_node = my_package.autoware_loop_node:main'
        ],
    },
)
