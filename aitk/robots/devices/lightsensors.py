# -*- coding: utf-8 -*-
# *************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2020 Calysto Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
#
# *************************************

import math

from ..colors import PURPLE, YELLOW
from ..utils import distance


class LightSensor:
    def __init__(self, position=(0, 0), name="light", **kwargs):
        config = {
            "position": position,
            "name": name,
        }
        self.robot = None
        self.initialize()
        self.from_json(config)

    def __repr__(self):
        return "<LightSensor %r position=%r>" % (self.name, self.position,)

    def initialize(self):
        self.type = "light"
        self.name = "light"
        self.value = 0.0
        # FIXME: add to config
        self.multiplier = 1000  # CM
        self.position = [0, 0]
        self.dist_from_center = distance(0, 0, self.position[0], self.position[1])
        self.dir_from_center = math.atan2(-self.position[0], self.position[1])

    def from_json(self, config):
        if "name" in config:
            self.name = config["name"]
        if "position" in config:
            self.position = config["position"]
            # Get location of sensor, doesn't change once position is set:
            self.dist_from_center = distance(0, 0, self.position[0], self.position[1])
            self.dir_from_center = math.atan2(-self.position[0], self.position[1])

    def to_json(self):
        config = {
            "class": self.__class__.__name__,
            "position": self.position,
            "name": self.name,
        }
        return config

    def step(self, time_step):
        pass

    def update(self, draw_list=None):
        self.value = 0
        # Location of sensor:
        p = self.robot.rotate_around(
            self.robot.x,
            self.robot.y,
            self.dist_from_center,
            self.robot.direction + self.dir_from_center + math.pi / 2,
        )
        for bulb in self.robot.world.bulbs:  # for each light source:
            x, y, z, brightness, light_color = (  # noqa: F841
                bulb.x,
                bulb.y,
                bulb.z,
                bulb.brightness,
                bulb.color,
            )
            # FIXME: use bulb_color for filter?

            angle = math.atan2(x - p[0], y - p[1])
            dist = distance(x, y, p[0], p[1])
            hits = self.robot.cast_ray(p[0], p[1], angle, dist)
            if self.robot.world.debug and draw_list is not None:
                draw_list.append(("draw_circle", (p[0], p[1], 2)))
                draw_list.append(("draw_circle", (x, y, 2)))

                for hit in hits:
                    draw_list.append(("set_fill_style", (PURPLE,)))
                    draw_list.append(("draw_circle", (hit.x, hit.y, 2)))

            if len(hits) == 0:  # nothing blocking! we can see the light
                # Make sure distance not zero:
                dist = max(dist, 0.001)
                # Maximum value of 100.0 with defaults:
                self.value += min(
                    brightness * self.multiplier / (dist ** 2), self.multiplier / 10
                )
                if draw_list is not None:
                    draw_list.append(("strokeStyle", (PURPLE, 1)))
                    draw_list.append(("draw_line", (x, y, p[0], p[1])))

    def draw(self, backend):
        backend.set_fill_style(YELLOW)
        backend.draw_circle(self.position[0], self.position[1], 2)

    def get_reading(self):
        """
        Get the light reading from the sensor.
        """
        return self.value

    def watch(self, title="Light Sensor:"):
        from ..watchers import AttributesWatcher

        if self.robot is None or self.robot.world is None:
            print("ERROR: can't watch until added to robot, and robot is in world")
            return None

        watcher = AttributesWatcher(
            self, "name", "value", title=title, labels=["Name:", "Light:"]
        )
        self.robot.world.watchers.append(watcher)
        return watcher.widget

    def set_position(self, position):
        """
        Set the position of the light sensor with respect to the center of the
        robot.

        Args:
            * position: (list/tuple of length 2) represents [x, y] in CM from
                center of robot
        """
        if len(position) != 2:
            raise ValueError("position must be of length two")

        self.position = position
        # Get location of sensor, doesn't change once position is set:
        self.dist_from_center = distance(0, 0, self.position[0], self.position[1])
        self.dir_from_center = math.atan2(-self.position[0], self.position[1])
