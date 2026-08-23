from setuptools import setup

package_name = 'uned_kheperaiv_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={package_name: ['main.ui', 'figs/*']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco José Mañas Álvarez',
    maintainer_email='fjmanas@dia.uned.es',
    description='PyQt graphical interface for handling a single Khepera IV robot',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface_node = uned_kheperaiv_gui.interface_gui:main'
        ],
    },
)
