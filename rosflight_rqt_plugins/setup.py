import os
from setuptools import setup

package_name = 'rosflight_rqt_plugins'

setup(
    name=package_name,
    version='1.0.0',
    # Packages to export
    package_dir={'': 'src'},
    packages=['param_tuning'],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resources/' + package_name]),
        # Include package files
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), ['plugin.xml']),
        (os.path.join('share', package_name, 'resources'), ['resources/param_tuning.ui']),
        (os.path.join('lib', package_name), ['scripts/param_tuning']),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jacob Moore, Joseph Richie, Brandon Sutherland',
    maintainer='Brandon Sutherland',
    maintainer_email='bsuther2@byu.edu',
    keywords=['ROSflight', 'rqt'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD',
        'Programming Language :: Python',
        'Topic :: Ground Control Stations',
    ],
    description='A collection of rqt plugins for ROSflight',
    license='BSD-3-Clause',
)
