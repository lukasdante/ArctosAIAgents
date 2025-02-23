from setuptools import find_packages, setup

package_name = 'aiagent'

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
    maintainer='louis',
    maintainer_email='johnlouis.dante@gmail.com',
    description='The ROS 2 Humble implementation of Arctos robot arm.',
    license='Unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder = aiagent.recorder:main',
            'talker = aiagent.talker:main',
            'writer = aiagent.writer:main',
            'agent = aiagent.agents:main',
        ],
    },
)
