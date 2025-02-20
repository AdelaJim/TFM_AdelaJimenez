from setuptools import find_packages, setup

package_name = 'trayectories'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alvaro',
    maintainer_email='miguel.ler.alonso@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "move_joints = trayectories.move_joints:main",
            "move_l = trayectories.move_l:main",
            "move_l_original = trayectories.move_l_original:main",
            "move_l_gcode = trayectories.move_l_gcode:main"
            # "node_launcher = trayectories.node_launcher:main"kk,
        ],
    },
)
