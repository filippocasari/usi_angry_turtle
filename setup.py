from setuptools import setup

package_name = 'usi_angry_turtle'

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
    maintainer='usi',
    maintainer_email='usi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'point1=usi_angry_turtle.point1:main',
            'point2=usi_angry_turtle.point2:main',
            'point3=usi_angry_turtle.point3:main',
            'point4=usi_angry_turtle.point4:main'
        ],
    },
)
