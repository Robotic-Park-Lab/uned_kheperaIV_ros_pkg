from setuptools import setup

package_name = 'uned_kheperaiv_driver'

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
    maintainer='kiko-hp-omen',
    maintainer_email='fma527@ual.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kheperaIV_client_driver = uned_kheperaiv_driver.kheperaIV_client_driver:main'
        ],
    },
)
