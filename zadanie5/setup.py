from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'zadanie5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maciej',
    maintainer_email='macieksw2000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'pos_service = zadanie5.oint:main',
        	'pos_client = zadanie5.ocmd:main',
        	'ikin = zadanie5.ikin:main',
        ],
    },
)
