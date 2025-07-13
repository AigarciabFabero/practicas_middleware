from setuptools import setup

package_name = 'pub_sub_node'

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
    maintainer='tu_usuario',
    maintainer_email='tu@email.com',
    description='Nodo combinado Publicador/Suscriptor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_sub_node = pub_sub_node.pub_sub_node:main',
        ],
    },
)