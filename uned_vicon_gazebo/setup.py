from setuptools import setup

package_name = 'uned_vicon_gazebo'

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
            'vicon_gazebo = uned_vicon_gazebo.vicon_gazebo:main'
        ],
    },
)
