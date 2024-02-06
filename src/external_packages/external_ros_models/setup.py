from setuptools import find_packages, setup
import glob

package_name = 'external_ros_models'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # URDF:
        (f'share/{package_name}/urdf/bin', glob.glob('urdf/bin/*', recursive=True)),
        (f'share/{package_name}/urdf/robot_pedestal', glob.glob('urdf/robot_pedestal/*', recursive=True)),

        # Meshes:
        (f'share/{package_name}/meshes/bin/collision', glob.glob('meshes/bin/collision/*', recursive=True)),
        (f'share/{package_name}/meshes/bin/visual', glob.glob('meshes/bin/visual/*', recursive=True)),
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
