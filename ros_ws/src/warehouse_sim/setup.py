from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'warehouse_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *[(f'share/{package_name}/{Path(path).parent}', [path])
          for path in sorted(glob('maps/**/*.yaml', recursive=True))
          if Path(path).is_file()],
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='dima',
    maintainer_email='dima@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map = warehouse_sim.map_node:main',
        ],
    },
)
