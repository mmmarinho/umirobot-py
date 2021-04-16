"""
Copyright (C) 2021 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
import setuptools
from setuptools import setup

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name='umirobot',
    version_format='{tag}.{commitcount}',
    setup_requires=['setuptools-git-version'],
    install_requires=[
        'pyserial', 'PyQt5', 'qdarkstyle', 'dqrobotics'
    ],
    packages=setuptools.find_packages(),
    url='https://github.com/mmmarinho/umirobot-py',
    license='GPLv3',
    author='Murilo M. Marinho',
    author_email='murilo@g.ecc.u-tokyo.ac.jp',
    description='UMIRobot control code for Python.',
    long_description=long_description,
    long_description_content_type="text/markdown",
    python_requires='>=3.8',
    package_data={'umirobot.gui': ['icon.png', 'umirobot.png']},
    include_package_data=True
)
