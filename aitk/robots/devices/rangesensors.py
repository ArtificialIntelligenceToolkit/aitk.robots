# -*- coding: utf-8 -*-
# ************************************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2021 AITK Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
# ************************************************************

import math

from ..utils import (
    display,
    Color,
    arange,
    distance,
    degrees_to_world,
    rotate_around,
    world_to_degrees,
)
from .base import BaseDevice

class RangeSensor(BaseDevice):
    def __init__(
        self, position=(8, 0), a=0, max=20, width=1.0, name="sensor", **kwargs
    ):
        """
        A range sensor that reads "reading" when no obstacle has been
        detected. "reading" is a ratio of distance/max, and "distance"
        is the reading in CM.

        Args:
            * position: (int, int) the location on the robot in (x, y)
            * a: (number) the direction in degrees the sensor is
                facing.
            * max: (number) max distance in CM that the range sensor can sense
            * width: (number) 0 for laser, or wider for sonar
            * name: (str) the name of the sensor
        """
        config = {
            "position": position,
            "a": a, # degrees in the config file
            "max": max,
            "width": width,
            "name": name,
        }
        config.update(kwargs)
        self._watcher = None
        self.robot = None
        self.initialize()
        self.from_json(config)

    def initialize(self):
        self.type = "ir"
        self.time = 0.0
        self.reading = 1.0
        self.position = [8, 0]
        self.dist_from_center = distance(0, 0, self.position[0], self.position[1])
        self.dir_from_center = math.atan2(-self.position[0], self.position[1])
        self.a = degrees_to_world(0)  # comes in degrees, save as radians
        self.max = 20  # CM
        self.width = 1.0  # radians
        self.name = "sensor"
        self.distance = self.reading * self.max

    def watch(self, title="Range Sensor:"):
        widget = self.get_widget(title=title)
        return display(widget)

    def get_widget(self, title=None, attributes=None):
        from ..watchers import AttributesWatcher

        if self.robot is None or self.robot.world is None:
            print("ERROR: can't watch until added to robot, and robot is in world")
            return None

        if self._watcher is None:
            title = title if title is not None else "Range Sensor:"
            attributes = attributes if attributes is not None else "all"
            self._watcher = AttributesWatcher(
                self,
                "name",
                "reading",
                "distance",
                title=title,
                labels=["Name:", "Reading:", "Distance:"],
                attributes=attributes,
            )
            self.robot.world._watchers.append(self._watcher)
        else:
            self._watcher.set_arguments(title=title, attributes=attributes)
            self._watcher.update()
        return self._watcher.widget

    def from_json(self, config):
        valid_keys = set([
            "position", "a", "max", "width", "name", "class"
        ])
        self.verify_config(valid_keys, config)

        if "position" in config:
            self.position = config["position"]
            # Get location of sensor, doesn't change once position is set:
            self.dist_from_center = distance(0, 0, self.position[0], self.position[1])
            self.dir_from_center = math.atan2(-self.position[0], self.position[1])
        if "a" in config:
            self.a = degrees_to_world(config["a"])
        if "max" in config:
            self.max = config["max"]
        if "width" in config:
            self.width = config["width"] * math.pi / 180  # save as radians
            if self.width == 0:
                self.type = "laser"
        if "name" in config:
            self.name = config["name"]
        self.distance = self.reading * self.max

    def to_json(self):
        config = {
            "class": self.__class__.__name__,
            "position": self.position,
            "a": world_to_degrees(self.a),
            "max": self.max,
            "width": self.width * 180 / math.pi,  # save as degrees
            "name": self.name,
        }
        return config

    def __repr__(self):
        return "<RangeSensor %r a=%r, range=%r, width=%r, position=%r>" % (
            self.name,
            round(world_to_degrees(self.a), 1),
            self.max,
            round(self.width * 180 / math.pi, 1),
            self.position,
        )

    def _step(self, time_step):
        pass

    def update(self, draw_list=None):
        # Update timestamp:
        self.time = self.robot.world.time
        # This changes:
        p = rotate_around(
            self.robot.x,
            self.robot.y,
            self.dist_from_center,
            self.robot.a + self.dir_from_center + math.pi / 2,
        )

        if self.robot.world.debug and draw_list is not None:
            draw_list.append(("draw_ellipse", (p[0], p[1], 2, 2), {}))

        self.set_reading(1.0)
        if self.width != 0:
            for incr in arange(-self.width / 2, self.width / 2, self.width / 2):
                hits = self.robot.cast_ray(
                    p[0],
                    p[1],
                    -self.robot.a + math.pi / 2.0 + incr - self.a,
                    self.max,
                )
                if hits:
                    if self.robot.world.debug and draw_list is not None:
                        draw_list.append(
                            ("draw_ellipse", (hits[-1].x, hits[-1].y, 2, 2), {})
                        )
                    # Closest hit:
                    if hits[-1].distance < self.get_distance():
                        self.set_distance(hits[-1].distance)
        else:
            hits = self.robot.cast_ray(
                p[0],
                p[1],
                -self.robot.a + math.pi / 2.0 - self.a,
                self.max,
            )
            if hits:
                if self.robot.world.debug and draw_list is not None:
                    draw_list.append(("draw_ellipse", (hits[-1].x, hits[-1].y, 2, 2), {}))
                # Closest hit:
                if hits[-1].distance < self.get_distance():
                    self.set_distance(hits[-1].distance)

    def draw(self, backend):
        backend.set_fill(Color(128, 0, 128, 64))
        dist = self.get_distance()
        if self.width > 0:
            if self.get_reading() < 1.0:
                backend.strokeStyle(Color(255), 1)
            else:
                backend.strokeStyle(Color(0), 1)

            backend.draw_arc(
                self.position[0],
                self.position[1],
                dist,
                dist,
                self.a - self.width / 2,
                self.a + self.width / 2,
            )
        else:
            if self.get_reading() < 1.0:
                backend.strokeStyle(Color(255), 1)
            else:
                backend.strokeStyle(Color(128, 0, 128, 64), 1)

            backend.draw_line(
                self.position[0], self.position[1], dist + self.position[0], 0
            )

    def get_distance(self):
        """
        Get the last range distance of the sensor in CM. The distance is
        between 0 and max.
        """
        return self.distance

    def get_reading(self):
        """
        Get the last range reading of the sensor.  The reading is between
        0 and 1.
        """
        return self.reading

    def get_max(self):
        """
        Get the maximum distance in CM the sensor can sense.
        """
        return self.max

    def get_position(self):
        """
        Get the position of the sensor. This represents the location
        of the sensor in [x, y] CM.
        """
        return self.position

    def get_angle(self):
        """
        Get the direction in degrees. Use RangeSensor.a
        to get the raw radians.
        """
        return world_to_degrees(self.a)

    def get_width(self):
        """
        Get the width of the sensor in degrees. Use
        RangeSensor.width to see raw radians.
        """
        return self.width * 180 / math.pi

    def get_name(self):
        """
        Get the name of the range sensor.
        """
        return self.name

    def set_name(self, name):
        """
        Set the name of the range sensor.

        Args:
            * name: (str) the name of the range sensor
        """
        self.name = name

    def set_distance(self, distance):
        """
        Set the distance that the sensor is reading. You would not
        usually do this manually.

        Args:
            * distance: (number) distance in CM to sensed object
        """
        self.distance = distance
        self.reading = distance / self.max

    def set_reading(self, reading):
        """
        Set the reading that the sensor is reading. You would not
        usually do this manually.

        Args:
            * reading: (number) between 0 and 1
        """
        self.reading = reading
        self.distance = reading * self.max

    def set_max(self, max):
        """
        Set the maximum distance in CM that this sensor can sense.

        Args:
            * max: (number) max distance in CM the sensor can sense.
        """
        self.max = max

    def set_position(self, position):
        """
        Set the position of the sensor. position must be a
        list/tuple of length 2 representing [x, y] in CM of the
        location of the sensor relative to the center of the
        robot.

        Args:
            * position: (list of length 2 numbers) the location
                of the sensor in relationship to the center of the
                robot.
        """
        if len(position) != 2:
            raise ValueError("position must be of length two")

        self.position = position
        self.dist_from_center = distance(0, 0, self.position[0], self.position[1])
        self.dir_from_center = math.atan2(-self.position[0], self.position[1])

    def set_angle(self, a):
        """
        Set the direction of the sensor.

        Args:
            * a: (number) the angle of the direction of sensor in degrees
        """
        self.a = degrees_to_world(a)

    def set_width(self, width):
        """
        Set the width of the range sensor in degrees. 0 width
        is a laser range finder. Larger values indicate the
        width of an IR sensor. It is measured in three locations:
        start, middle, and stop. The value of the sensor is the
        minimum of the three.

        Args:
            * width: (number) angle in degrees
        """
        self.width = width * math.pi / 180  # save as radians
        if self.width == 0:
            self.type = "laser"
        else:
            self.type = "ir"
