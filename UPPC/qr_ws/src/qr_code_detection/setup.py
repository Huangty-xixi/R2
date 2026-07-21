from setuptools import find_packages, setup

package_name = 'qr_code_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/qr_code_detection']),
    ('share/qr_code_detection', ['package.xml']),
    ('share/qr_code_detection/launch', ['launch/qr_demo.launch.py']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ma',
    maintainer_email='ma@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'image_publisher = qr_code_detection.image_publisher:main',
        'qr_decoder = qr_code_detection.qr_decoder:main',
        'qr_select_from_file = qr_code_control.qr_select_from_file:main',
        'qrcode_control = qr_code_control.qrcode_control:main',

    ],
},

)
