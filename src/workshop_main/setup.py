from setuptools import find_packages, setup

package_name = 'workshop_main'

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
    maintainer='bgiabao',
    maintainer_email='baobui226mh@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = workshop_main.minimal_publisher:main',
            'minimal_subscriber = workshop_main.minimal_subscriber:main',
        ],
    },
)
