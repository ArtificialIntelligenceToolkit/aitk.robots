# -*- coding: utf-8 -*-
# ************************************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2021 AITK Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
# ************************************************************

import math

from ..utils import distance, rotate_around, Color


class Bulb:
    """
    Class representing lights in the world.
    """

    def __init__(self, color, x=0, y=0, z=0, brightness=1, name=None, **kwargs):
        """
        Create a lightbulb.
        """
        self.color = Color(color)
        self._x = x
        self._y = y
        self._z = z
        self.brightness = brightness
        self.name = name if name is not None else "bulb"
        self.draw_rings = 7 # rings around bulb light in views
        self._watcher = None
        self.robot = None
        self.initialize()

    @property
    def x(self):
        if self.robot is None:
            return self._x
        else:
            x, y = rotate_around(self.robot.x,
                                 self.robot.y,
                                 self.dist_from_center,
                                 self.robot.a + self.dir_from_center)
            return x
    @x.setter
    def x(self, value):
        self._x = value

    @property
    def y(self):
        if self.robot is None:
            return self._y
        else:
            x, y = rotate_around(self.robot.x,
                                 self.robot.y,
                                 self.dist_from_center,
                                 self.robot.a + self.dir_from_center)
            return y

    @y.setter
    def y(self, value):
        self._y = value

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, value):
        self._z = value

    def __repr__(self):
        return "<Bulb color=%r, x=%r, y=%r, z=%r, brightness=%r, name=%r>" % (
            self.color,
            self._x,
            self._y,
            self._z,
            self.brightness,
            self.name,
        )

    def initialize(self):
        self.type = "bulb"
        self.dist_from_center = distance(0, 0, self._x, self._y)
        self.dir_from_center = math.atan2(-self._x, self._y)

    def to_json(self):
        config = {
            "class": self.__class__.__name__,
            "color": str(self.color),
            "x": self._x,
            "y": self._y,
            "z": self._z,
            "brightness": self.brightness,
            "name": self.name,
        }
        return config

    def from_json(self, config):
        if "x" in config:
            self._x = config["x"]
        if "y" in config:
            self._y = config["y"]
        if "z" in config:
            self._z = config["z"]
        if "name" in config:
            self.name = config["name"]
        if "color" in config:
            self.color = Color(config["color"])
        if "brightness" in config:
            self.brightness = config["brightness"]
        self.initialize()

    def _step(self, time_step):
        pass

    def update(self, draw_list=None):
        pass

    def draw(self, backend):
        # World draws the lights? Pro: draws first
        pass

    def watch(self, title="Light Sensor:"):
        widget = self.get_widget(title=title)
        return display(widget)

    def get_widget(self, title="Light Sensor:"):
        from ..watchers import AttributesWatcher

        if self.robot is None or self.robot.world is None:
            print("ERROR: can't watch until added to robot, and robot is in world")
            return None

        if self._watcher is None:
            self._watcher = AttributesWatcher(
                self, "name", "brightness", title=title, labels=["Name:", "Brightness:"]
            )
            self.robot.world._watchers.append(self._watcher)

        return self._watcher.widget