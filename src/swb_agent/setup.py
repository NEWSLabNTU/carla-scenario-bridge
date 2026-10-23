from setuptools import setup

package_name = 'swb_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.adapters'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Agent SDK: drive scenario-world-bridge scenarios with any autopilot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swb-agent = swb_agent.cli:main',
            'ros_adapter = swb_agent.ros_adapter:main',
            'autoware_agent = swb_agent.adapters.autoware_ros:main',
            'vehicle_agent = swb_agent.adapters.vehicle_agent:main',
            'carla_basic = swb_agent.adapters.carla_basic:main',
        ],
    },
)
