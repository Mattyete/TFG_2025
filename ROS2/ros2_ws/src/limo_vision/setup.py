from setuptools import setup

package_name = 'limo_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=['scripts'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/keras_detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matt',
    maintainer_email='mayeteco7@gmail.com',
    description='Vision system using Keras for LIMO robot (ROS 2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keras_detector_v2 = scripts.keras_detector_v2:main',
        ],
    },
)
