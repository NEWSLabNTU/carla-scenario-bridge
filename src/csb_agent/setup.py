from setuptools import setup

package_name = 'csb_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Agent SDK: drive carla-scenario-bridge scenarios with any autopilot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csb-agent = csb_agent.cli:main',
            'ros_adapter = csb_agent.ros_adapter:main',
        ],
    },
)
