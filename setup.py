from setuptools import find_packages, setup
from glob import glob

package_name = 'trajectory_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir config
        (f'share/{package_name}/config', glob('config/*.yaml')),
        # Incluir launch
        (f'share/{package_name}/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asoriano',
    maintainer_email='asoriano@robotnik.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_follower = trajectory_follower.trajectory_follower:main',
        ],
    },
)
