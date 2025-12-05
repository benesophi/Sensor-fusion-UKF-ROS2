from setuptools import setup
import os
from glob import glob

package_name = 'kalmanfilter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sophia',
    maintainer_email='sophia@todo.todo',
    description='Filtro de Kalman com FilterPy para ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ukf_node = kalmanfilter.ukf_filterpy:main',
            'data_extraction_node = kalmanfilter.data_extraction:main',
            'plotter = kalmanfilter.plot_results:main'
        ],
    },
)

