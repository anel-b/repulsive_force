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
            'f_repulsion = repulsive_force.f_repulsion:main',
            'f_repulsion_v2 = repulsive_force.f_repulsion_v2:main',
        ],
    },
)
