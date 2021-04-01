# -*- coding: utf-8 -*-
# *************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2020 Calysto Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
#
# *************************************

from ._version import __version__  # noqa: F401
from .config import setup_backend, switch_backend  # noqa: F401
from .devices import Camera, GroundCamera, LightSensor, RangeSensor, SmellSensor  # noqa: F401
from .robot import Robot, Scribbler, Vehicle  # noqa: F401
from .utils import Color, gallery, load_world  # noqa: F401
from .world import Bulb, Wall, World  # noqa: F401

setup_backend()  # checks os.environ
