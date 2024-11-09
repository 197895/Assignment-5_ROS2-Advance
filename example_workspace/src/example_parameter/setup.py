from setuptools import find_packages, setup

package_name = 'example_parameter'

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
    maintainer='myubuntu',
    maintainer_email='protonnewbing@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_parameter_node = 相对于setup.py的路径？
            
            
            
            ###example_parameter.example_parameter_node:main
        ],
    },
)
