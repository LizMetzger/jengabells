from setuptools import setup

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/jenga_vision.launch.py',
                                   'launch/cv.launch.py',
                                   'config/april.rviz',
                                   'config/april.yaml',
                                   'config/tf.yaml',
                                   'model/keras_model.h5',
                                   'model/labels.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khughes1',
    maintainer_email='katiereeshughes@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam = camera.realsense:main',
            'cali = camera.calibrate:main',
            'broadcast = camera.broadcast_transform:main'
        ],
    },
)
