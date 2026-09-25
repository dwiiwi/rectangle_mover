from setuptools import find_packages, setup

package_name = 'rectangle_mover'

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
    maintainer='dwislam',
    maintainer_email='dwiaqidah3@gmail.com',
    description='ROS 2 node to move a robot in a rectangular path using TF feedback.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rectangle_node = rectangle_mover.rectangle_node:main'
        ],
    },
)
