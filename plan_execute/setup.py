from setuptools import setup

package_name = 'plan_execute'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/simple_move.launch.py',
                                   'plan_execute/plan_and_execute.py']),
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
            'simple_move=plan_execute.simple_move:test_entry',
            'cv_test=plan_execute.cv_test:test_entry'
        ],
    },
)
