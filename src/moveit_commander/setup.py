'''
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d["packages"] = ["moveit_commander"]
d["package_dir"] = {"": "src"}

setup(**d)
'''

import os
from glob import glob
# https://buildersbox.corp-sansan.com/entry/2019/12/09/110000
# from distutils.core import Extension
from setuptools import setup, Extension
from setuptools import find_packages

package_name = 'moveit_commander'

module = Extension(
    'python_interface',
    sources = ['/home/chaos/moveit2_ws/src/moveit2_tutorials/doc/moveit2_commander_ws/src/moveit_commander/python_interface/wrap_python_robot_interface.cpp'],
    include_dirs = ['/usr/local/include/boost'],        # boostのインクルードディレクトリ
    library_dirs = ['/usr/local/lib'],                  # boostのライブラリディレクトリ
    libraries = ['boost_python']                        # libboost_python.aをリンクする
)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaos',
    maintainer_email='chaos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    ext_modules = [module],
)
