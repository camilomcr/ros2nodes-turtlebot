from setuptools import setup

package_name = 'turtle_bot'

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
    maintainer='robotica',
    maintainer_email='robotica@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_bot_teleop = turtle_bot.turtle_bot_teleop:main",
            "turtle_bot_interface = turtle_bot.turtle_bot_interface:main",
            "turtle_bot_player= turtle_bot.turtle_bot_player:main"
        ],
    },
)
