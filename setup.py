from setuptools import setup

package_name = 'para_multi_cam_bridge'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'ardu_cam_pub = para_multi_cam_bridge.ardu_cam_pub:main',
        	'ardu_cam_sub = para_multi_cam_bridge.ardu_cam_sub:main',
        ],
    },
)
