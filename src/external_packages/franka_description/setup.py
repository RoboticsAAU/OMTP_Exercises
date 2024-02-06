from setuptools import find_packages, setup
import glob

package_name = 'franka_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Robots:
        (f'share/{package_name}/robots', glob.glob('robots/*', recursive=True)),
        # Meshes
        (f'share/{package_name}/meshes/collision', glob.glob('meshes/collision/*', recursive=True)),
        (f'share/{package_name}/meshes/visual', glob.glob('meshes/visual/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='victor.risager@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
