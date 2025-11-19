from setuptools import setup
import os
from glob import glob

package_name = 'vosk_ros2'

# Collect all model files recursively
def get_model_files():
    model_files = []
    model_dir = 'models'
    if os.path.exists(model_dir):
        for root, dirs, files in os.walk(model_dir):
            for file in files:
                model_files.append(os.path.join(root, file))
    return model_files

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=['vosk_node'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ] + [(os.path.join('share', package_name, os.path.dirname(f)), [f]) for f in get_model_files()],
    install_requires=['setuptools', 'vosk', 'sounddevice'],
    zip_safe=True,
    maintainer='Zhuravliov Pavel',
    maintainer_email='zhuravliov.pav@yandex.ru',
    description='ROS2 package for Vosk speech recognition',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vosk_node = vosk_node:main',
        ],
    },
)
