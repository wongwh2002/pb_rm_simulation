import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'small_ramp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Correct path to include launch files
        (os.path.join('share', package_name, 'launch'), glob('small_ramp/launch/*.launch.py')),  # Updated line
        (os.path.join('share', package_name, 'urdf'), glob('small_ramp/urdf/*')),  # Add this line
        (os.path.join('share', package_name, 'worlds'), glob('small_ramp/worlds/*')),   
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wsentry',
    maintainer_email='wengwonghong5686@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = small_ramp.my_node:main',
            # This should point to the correct path
            'bringup_sim = small_ramp.launch.bringup_sim:main',
        ],
    },
)
