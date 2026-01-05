from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'input_output_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.xml'))),

        # 安装 config 文件
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input_filter = input_output_filter.input_filter:main',
            'output_filter = input_output_filter.output_filter:main'
        ],
    },
)
