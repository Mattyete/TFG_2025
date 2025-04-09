from setuptools import setup

package_name = 'limo_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matt',
    maintainer_email='mayeteco7@gmail.com',
    description='Vision with Keras for LIMO in ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keras_detector_v2 = limo_vision.keras_detector_v2:main',
        ],
    },
)
