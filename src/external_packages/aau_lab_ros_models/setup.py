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

        # URDF:
        (f'share/{package_name}/urdf/eu_pallet', glob.glob('urdf/eu_pallet/*.urdf.xacro', recursive=True)),
        (f'share/{package_name}/urdf/festo_bypass_module', glob.glob('urdf/festo_bypass_module/*.urdf.xacro', recursive=True)),
        (f'share/{package_name}/urdf/festo_robot_module', glob.glob('urdf/festo_robot_module/*.urdf.xacro', recursive=True)),
        (f'share/{package_name}/urdf/festo_straight_module', glob.glob('urdf/festo_straight_module/*.urdf.xacro', recursive=True)),
        (f'share/{package_name}/urdf/festo_t_module', glob.glob('urdf/festo_t_module/*.urdf.xacro', recursive=True)),
        (f'share/{package_name}/urdf/fib14', glob.glob('urdf/fib14/*.urdf.xacro', recursive=True)),
        (f'share/{package_name}/urdf/pool_cue', glob.glob('urdf/pool_cue/*.urdf.xacro', recursive=True)),
        (f'share/{package_name}/urdf/wood_box', glob.glob('urdf/wood_box/*.urdf.xacro', recursive=True)),

        # Meshes:
        (f'share/{package_name}/meshes/eu_pallet', glob.glob('meshes/eu_pallet/*', recursive=True)),
        (f'share/{package_name}/meshes/festo_bypass_module', glob.glob('meshes/festo_bypass_module/*', recursive=True)),
        (f'share/{package_name}/meshes/festo_robot_module', glob.glob('meshes/festo_robot_module/*', recursive=True)),
        (f'share/{package_name}/meshes/festo_straight_module', glob.glob('meshes/festo_straight_module/*', recursive=True)),
        (f'share/{package_name}/meshes/festo_t_module', glob.glob('meshes/festo_t_module/*', recursive=True)),
        (f'share/{package_name}/meshes/fib14', glob.glob('meshes/fib14/*', recursive=True)),
        (f'share/{package_name}/meshes/pool_cue', glob.glob('meshes/pool_cue/*', recursive=True)),
        (f'share/{package_name}/meshes/wood_box', glob.glob('meshes/wood_box/*', recursive=True)),
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
