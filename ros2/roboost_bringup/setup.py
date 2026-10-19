from glob import glob

from setuptools import setup

package_name = 'roboost_bringup'
share = 'share/' + package_name

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml']),
        *[(f'{share}/{d}', glob(f'{d}/*')) for d in ('launch', 'urdf', 'meshes', 'config', 'rviz', 'maps')],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jakob Friedl',
    maintainer_email='friedl.jak@gmail.com',
    description='Roboost robot description, bringup launch file and wheel odometry relay.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['odom_tf = roboost_bringup.odom_tf:main']},
)
