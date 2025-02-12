from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gripper_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/**')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        (os.path.join('share', package_name, 'config'), glob('config/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cimatec',
    maintainer_email='davidsilva29@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

