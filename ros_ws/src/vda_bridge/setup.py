from glob import glob
from setuptools import setup, find_packages
# Wire model and official schemas are installed with this ROS package.
setup(name='vda_bridge',version='0.6.0',
 packages=find_packages(),
 package_data={'protocols.vda5050':['schemas/*']},
 data_files=[('share/ament_index/resource_index/packages',['resource/vda_bridge']),
             ('share/vda_bridge',['package.xml']),('share/vda_bridge/config',glob('config/*.yaml'))],
 install_requires=['setuptools','PyYAML','jsonschema>=4.18,<5','paho-mqtt>=2.1,<3'],zip_safe=False,
 maintainer='Warehouse RMS contributors',maintainer_email='maintainer@example.com',
 description='Restricted VDA 5050 2.1.0 profile over MQTT',license='Apache-2.0',
 entry_points={'console_scripts':['robot_adapter = vda_bridge.robot_adapter:main']})
