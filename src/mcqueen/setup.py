from setuptools import setup

package_name = 'mcqueen'

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
    maintainer='camilo',
    maintainer_email='c.morillo@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mcqueen_teleop = mcqueen.mcqueen_teleop:main",
            "mcqueen_interface = mcqueen.mcqueen_interface:main",
	        "mcqueen_player= mcqueen.mcqueen_player:main"
        ],
    },
)
