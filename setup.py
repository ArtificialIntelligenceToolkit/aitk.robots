# -*- coding: utf-8 -*-
# **************************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2021 ArtificialIntelligenceToolkit
#
# https://github.com/ArtificialIntelligenceToolkit/
#
# **************************************************

"""
aitk.robots setup
"""
import io
import os

import setuptools

HERE = os.path.abspath(os.path.dirname(__file__))

# The name of the project
name = "aitk.robots"


# Get our version
def get_version(file, name="__version__"):
    """Get the version of the package from the given file by
    executing it and extracting the given `name`.
    """
    path = os.path.realpath(file)
    version_ns = {}
    with io.open(path, encoding="utf8") as f:
        exec(f.read(), {}, version_ns)
    return version_ns[name]


version = get_version(os.path.join(HERE, "aitk/robots/_version.py"))

with open(os.path.join(HERE, "README.md"), "r") as fh:
    long_description = fh.read()

setup_args = dict(
    name=name,
    version=version,
    url="https://github.com/ArtificialIntelligenceToolkit/%s" % name,
    author="Douglas Blank",
    description="A lightweight Python Robot simulator",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=setuptools.find_namespace_packages(include=['aitk.*']),
    package_data={"aitk.robots": ["worlds/*.json", "worlds/*.png"]},
    install_requires=["setuptools", "Pillow", "aitk.utils>=0.6.1", "ipywidgets"],
    python_requires=">=3.6",
    license="BSD-3-Clause",
    platforms="Linux, Mac OS X, Windows",
    keywords=["robot", "simulator", "jupyter", "python"],
    classifiers=[
        "License :: OSI Approved :: BSD License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Framework :: Jupyter",
    ],
)

if __name__ == "__main__":
    setuptools.setup(**setup_args)
