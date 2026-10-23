from setuptools import setup

PAQUET = 'x3_base'
setup(
    name=PAQUET,
    version='0.1.0',
    packages=[PAQUET],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PAQUET]),
        ('share/' + PAQUET, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zubulmuk92',
    maintainer_email='73728454+zubulmuk92@users.noreply.github.com',
    description="Pilote de la carte STM32 du ROSMASTER X3 : consignes de vitesse, odométrie des roues, IMU, batterie",
    license='Apache-2.0',
    entry_points={'console_scripts': ['base_node = x3_base.base_node:main']},
    tests_require=['pytest'],
)
