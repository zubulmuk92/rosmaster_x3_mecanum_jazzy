from setuptools import setup

PAQUET = 'x3_teleop'
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
    description="Conduite à la main du ROSMASTER X3 : clavier, manette",
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
