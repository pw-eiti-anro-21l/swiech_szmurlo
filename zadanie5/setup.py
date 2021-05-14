from setuptools import setup

package_name = 'zadanie5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        	'pos_service = zadanie5.oint_control_srv:main',
        	'pos_client = zadanie5.oint:main',
        	'ikin = zadanie5.ikin:main',
        ],
    },
)