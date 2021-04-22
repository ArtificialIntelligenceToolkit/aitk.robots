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
from .base import BaseDevice

class Bulb(BaseDevice):
    """
    Class representing lights in the world.
    """
    def __init__(self, color, x=0, y=0, z=0, brightness=50, name=None, **kwargs):
        """
        Create a lightbulb.

        Args:
            color: (Color) the color of the bulb
            x: (int) the position of bulb on x dimension
            y: (int) the position of bulb on y dimension
            z: (float) the height of bulb from ground (range 0 to 1)
            brightness: (number) the brightness of the bulb (default 50)
            name: (str) the name of the bulb
        """
        config = {
            "color": color,
            "x": x,
            "y": y,
            "z": z,
            "brightness": brightness,
            "name": name,
        }
        config.update(kwargs)
        self.state = "on"
        self.draw_rings = 7 # rings around bulb light in views
        self._watcher = None
        self.robot = None
        self.from_json(config)

    def initialize(self):
        self.type = "bulb"
        self.state = "on"
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
        valid_keys = set(["x", "y", "z", "name", "color",
                          "brightness", "class"])
        self.verify_config(valid_keys, config)

        if "x" in config:
            self._x = config["x"]
        if "y" in config:
            self._y = config["y"]
        if "z" in config:
            self._z = config["z"]

        if "name" in config:
            name = config["name"]
        else:
            name = None
        self.name = name if name is not None else "bulb"

        if "color" in config:
            self.color = Color(config["color"])
        if "brightness" in config:
            self.brightness = config["brightness"]
        self.initialize()

    def on(self):
        self.state = "on"
        if self.robot is not None and self.robot.world is not None:
            self.robot.world.update()

    def off(self):
        self.state = "off"
        if self.robot is not None and self.robot.world is not None:
            self.robot.world.update()

    @property
    def x(self):
        if self.robot is None:
            return self._x
        else:
            if self.dist_from_center != 0:
                x, y = rotate_around(0,
                                     0,
                                     self.dist_from_center,
                                     self.robot.a + self.dir_from_center)
                return x
            else:
                return 0

    @x.setter
    def x(self, value):
        self._x = value

    @property
    def y(self):
        if self.robot is None:
            return self._y
        else:
            if self.dist_from_center != 0:
                x, y = rotate_around(0,
                                     0,
                                     self.dist_from_center,
                                     self.robot.a + self.dir_from_center)
                return y
            else:
                return 0

    def get_position(self, world=True):
        if self.robot is None:
            return self._x, self._y
        else:
            if world:
                if self.dist_from_center != 0:
                    x, y = rotate_around(self.robot.x,
                                         self.robot.y,
                                         self.dist_from_center,
                                         self.robot.a + self.dir_from_center)
                    return x, y
                else:
                    return self.robot.x, self.robot.y
            else: # local to robot
                if self.dist_from_center != 0:
                    x, y = rotate_around(0,
                                         0,
                                         self.dist_from_center,
                                         self.robot.a + self.dir_from_center)
                    return x, y
                else:
                    return 0, 0

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

    def _step(self, time_step):
        pass

    def update(self, draw_list=None):
        pass

    def draw(self, backend):
        if self.state == "off":
            return
        color = self.color
        backend.line_width = 0
        backend.noStroke()
        for i in range(self.draw_rings):
            radius = (self.draw_rings - i) * 2
            color.alpha = (i + 1)/self.draw_rings * 255
            backend.set_fill_style(color)
            x, y = self.get_position(world=False)
            backend.draw_circle(self.x, self.y, self.brightness/30 * radius)
        backend.line_width = 1

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
