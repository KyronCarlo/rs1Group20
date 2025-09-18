from setuptools import setup
package_name = 'drone_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='rqt plugin with drone control buttons',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'drone_ui = drone_ui.plugin:standalone_main',  # optional
        ],
    },
)