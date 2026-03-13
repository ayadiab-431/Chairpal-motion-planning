from setuptools import find_packages, setup

package_name = 'medical_wheelchair'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_wheelchair.launch.py', 'launch/mapper.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/medical_wheelchair.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/wheelchair.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayadiab',
    maintainer_email='diabaya695@gmail.com',
    description='Autonomous Medical Wheelchair Simulation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tf_republisher = medical_wheelchair.tf_republisher:main',
        ],
    },
)
