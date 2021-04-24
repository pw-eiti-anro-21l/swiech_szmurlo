from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages


package_name = 'zadanie3'

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
    maintainer='piotr',
    maintainer_email='piotr.szmurlo7@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'kdl_dkin = zadanie3.KDL_DKIN:main',
        'nonkdl_dkin = zadanie3.NONKDL_DKIN:main',
        ],
    },
)
