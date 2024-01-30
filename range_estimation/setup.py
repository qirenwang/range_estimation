from setuptools import find_packages, setup
import os

package_name = 'range_estimation'

# Helper function to automatically find all files within a directory
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

# Include all files from the 'data', 'models', and 'scaler' directories
data_files = package_files('range_estimation/data')
model_files = package_files('range_estimation/models')
scaler_files = package_files('range_estimation/scaler')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add data, model, and scaler files to the installation
        (os.path.join('share', package_name, 'data'), data_files),
        (os.path.join('share', package_name, 'models'), model_files),
        (os.path.join('share', package_name, 'scaler'), scaler_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qiren',
    maintainer_email='qiren@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = range_estimation.publisher:main',
            'subscriber = range_estimation.subscriber:main',
        ],
    },
)
