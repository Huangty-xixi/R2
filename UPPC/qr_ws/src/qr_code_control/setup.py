from setuptools import find_packages, setup

package_name = 'qr_code_control'

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
        'qrcode_control = qr_code_control.qrcode_control:main',
        'menu_input = qr_code_control.menu_input:main',
        'qr_select_from_file = qr_code_control.qr_select_from_file:main',
    ],
},



)
