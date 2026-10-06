from setuptools import setup
setup(name='hri_bai3_perception', version='0.1.0', packages=['hri_bai3_perception'],
 data_files=[('share/ament_index/resource_index/packages',['resource/hri_bai3_perception']),('share/hri_bai3_perception',['package.xml'])],
 install_requires=['setuptools'], zip_safe=True, maintainer='Hung', maintainer_email='hung@example.com',
 description='Camera-only RGB-D cube perception', license='Apache-2.0',
 entry_points={'console_scripts':['perception = hri_bai3_perception.node:main']})
