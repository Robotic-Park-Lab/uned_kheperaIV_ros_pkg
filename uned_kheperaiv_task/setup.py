from setuptools import setup

package_name = 'uned_kheperaiv_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiko',
    maintainer_email='fjmanas@dia.uned.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shape_based_formation_control = uned_kheperaiv_task.shape_based_formation_control:main',
            'distance_based_formation_control = uned_kheperaiv_task.distance_based_formation_control:main',
            'gazebo_driver = uned_kheperaiv_task.gazebo_driver:main'
        ],
    },
)
