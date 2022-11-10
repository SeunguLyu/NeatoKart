from setuptools import setup

package_name = 'neato_kart'

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
    maintainer='seungu',
    maintainer_email='travislyu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'build_circuit = neato_kart.build_circuit:main',
            'detect_april_tag = neato_kart.detect_april_tag:main'
        ],
    },
)
