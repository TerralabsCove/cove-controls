from setuptools import setup
from glob import glob
import os

package_name = 'cove_kiosk_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), ['web/index.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='HTTP kiosk bridge and order orchestrator for the COVE arm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'kiosk_bridge = cove_kiosk_bridge.kiosk_bridge_node:main',
        ],
    },
)
