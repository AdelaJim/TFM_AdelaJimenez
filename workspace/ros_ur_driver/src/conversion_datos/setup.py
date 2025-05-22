from setuptools import find_packages, setup

package_name = 'conversion_datos'

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
    maintainer='Adela',
    maintainer_email='adelajimenezhervas@gmail.com',
    description='Este paquete se encarga de realizar diferentes conversiones de datos para su posterios ejecución',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convertir_en_gcode = conversion_datos.convertir_en_gcode:main',
            'convertir_en_articulares = conversion_datos.convertir_en_articulares:main'

        ],
    },
)
