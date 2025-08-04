from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'trash_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/trash_system.launch.py']),
#        ('share/' + package_name, glob(os.path.join('trash_project', '*.pt'))),
	    ('share/' + package_name + '/models', glob(os.path.join(package_name, 'models', '*.pt'))),
        ('share/' + package_name, glob(os.path.join('trash_project', '*.mp4'))),
        ('share/' + package_name, glob(os.path.join('trash_project', '*.jpg'))),
    ],
    install_requires=[
    	'setuptools',
    	'pyserial',
    	'opencv-python',
    	'ultralytics',
    	'numpy',
    	'torch',
    	'torchvision',
    	'scipy',
    	'matplotlib',
    	'pyyaml',
    ],
    zip_safe=True,
    maintainer='slashing',
    maintainer_email='slashing@todo.todo',
    description='TRASH; Senior Design Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'basic_webcam_node = trash_project.basic_webcam_node:main',
		'cv_node = trash_project.cv_node:main',
        'motor_node = trash_project.motor_node:main',
        'spectrometer_node = trash_project.spectrometer_node:main',
		'basic_motor_node = trash_project.basic_motor_node:main',
		'basic_cv_node = trash_project.basic_cv_node:main',
		'canny_node = trash_project.canny_node:main',
		'gui_node = trash_project.gui_node:main',
	],
    },
)
