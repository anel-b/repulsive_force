from setuptools import find_packages, setup

package_name = 'repulsive_force'

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
    maintainer='anel-b',
    maintainer_email='anybajrektarevic@gmail.com',
    description='Repulsive force computation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'repulsive_force_zed = repulsive_force.repulsive_force_zed:main',
            'repulsive_force_ply = repulsive_force.repulsive_force_ply:main',
        ],
    },
)
