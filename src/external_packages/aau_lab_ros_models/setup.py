from setuptools import find_packages, setup
import glob

package_name = 'aau_lab_ros_models'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/urdf', glob.glob('urdf/**/*.*.*', recursive=True)),
        (f'share/{package_name}/meshes', glob.glob('meshes/**/*.dae', recursive=True)),
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
