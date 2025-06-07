from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mobile'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuneyoungjun',
    maintainer_email='0204jun@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "particle_filter=mobile.imporved_particle:main",
            "parcking=mobile.parcking:main",
            "plot=mobile.plot:main",
            "plot_dock=mobile.plot_dock:main",
            "waypoint=mobile.waypoint:main",
            "waypoint_pub=mobile.waypoint_pub:main",
            "rein=mobile.rein:main",
            "avoid=mobile.avoid:main"
        ],
    },
)
