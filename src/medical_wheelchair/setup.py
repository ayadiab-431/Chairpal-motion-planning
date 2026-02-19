from setuptools import find_packages, setup

package_name = 'medical_wheelchair'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/medical_wheelchair/launch', ['launch/spawn_wheelchair.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayadiab',
    maintainer_email='diabaya695@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
