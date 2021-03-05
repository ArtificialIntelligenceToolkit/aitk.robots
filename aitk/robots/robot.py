# -*- coding: utf-8 -*-
# *************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2020 Calysto Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
#
# *************************************

import importlib
import math
import re

from .datasets import get_dataset
from .hit import Hit
from .utils import (
    Color,
    Line,
    Point,
    distance,
    intersect,
    intersect_hit,
    PI_OVER_180,
    PI_OVER_2,
    ONE80_OVER_PI,
    TWO_PI,
)


class Robot:
    """
    The base robot class.
    """

    def __init__(
        self,
        x=0,
        y=0,
        direction=0,
        color="red",
        name="Robbie",
        do_trace=True,
        height=0.25,
        max_trace_length=10,
        **kwargs
    ):
        """
        Base robot class. You'll need to define a body.
        """
        config = {
            "x": x,
            "y": y,
            "direction": direction,
            "color": color,
            "name": name,
            "do_trace": do_trace,
            "height": height,
            "max_trace_length": max_trace_length,
        }
        for item in [
            "state",
            "va",
            "vx",
            "vy",
            "tva",
            "tvx",
            "tvy",
            "va_max",
            "vx_max",
            "vy_max",
            "va_ramp",
            "vx_ramp",
            "vy_ramp",
            "image_data",
            "body",
            "devices",
        ]:
            arg = kwargs.pop(item, None)
            if arg is not None:
                config[item] = arg
        if len(kwargs) != 0:
            raise AttributeError(
                "unknown arguments for Robot: %s" % list(kwargs.keys())
            )

        self.world = None
        self._devices = []
        self.initialize()
        self.from_json(config)

    def __getitem__(self, item):
        if isinstance(item, int):
            return self._devices[item]
        elif isinstance(item, str):
            type_map = {}  # mapping of base to count
            search_groups = re.match(r"(.*)-(\d*)", item)
            if search_groups:
                search_type = search_groups[1].lower()
                if search_groups[2].isdigit():
                    search_index = int(search_groups[2])
                else:
                    search_type = item
                    search_index = 1
            else:
                search_type = item.lower()
                search_index = 1
            for device in self._devices:
                # update type_map
                device_type = device.type.lower()
                device_name = device.name.lower()
                device_index = None
                if "-" in device_type:
                    device_prefix, device_index = device_type.rsplit("-", 1)
                    if device_index.isdigit():
                        device_type = device_prefix
                        device_index = int(device_index)
                    else:
                        device_index = 1
                if device_type not in type_map:
                    type_map[device_type] = 1
                else:
                    type_map[device_type] += 1
                if device_index is None:
                    device_index = type_map[device_type]
                if search_type == device_type and search_index == device_index:
                    return device
                if search_type == device_name:
                    return device
        return None

    def __len__(self):
        return len(self._devices)

    def __repr__(self):
        if self.world is None:
            return "<Robot(name=%r, unconnected)>" % (self.name,)
        else:
            return "<Robot(name=%r, position=%s,%s,%s v=%s,%s,%s)>" % (
                self.name,
                round(self.x, 2),
                round(self.y, 2),
                round(((self.direction + math.pi)* (ONE80_OVER_PI)) % 360, 2),
                round(self.vx, 2),
                round(self.vy, 2),
                round(self.va, 2),
            )

    def initialize(self):
        """
        Initialize the robot properties.
        """
        self.name = "Robbie"
        self.state = {}
        self._set_color("red")
        self.do_trace = True
        self.trace = []
        self.text_trace = []
        self.pen_trace = []
        self.pen = (None, 0)
        self.body = []
        self.max_trace_length = 10  # seconds
        self.x = 0  # cm
        self.y = 0  # cm
        self.height = 0.25
        self.direction = 0  # radians
        self.vx = 0.0  # velocity in x direction, CM per second
        self.vy = 0.0  # velocity in y direction, degrees per second
        self.va = 0.0  # turn velocity
        self.tvx = 0.0
        self.tvy = 0.0
        self.tva = 0.0
        self.va_ramp = 1.0  # seconds to reach max speed
        self.vx_ramp = 1.0  # seconds to reach max speed
        self.vy_ramp = 1.0  # seconds to reach max speed
        self.vx_max = 2.0  # CM/SEC
        self.va_max = math.pi * 0.90  # RADIANS/SEC
        self.vy_max = 2.0  # CM/SEC
        self.stalled = False
        # Slight box around center, in case no
        # body:
        self.bounding_lines = [
            Line(Point(-1, -1), Point(1, -1)),
            Line(Point(1, -1), Point(1, 1)),
            Line(Point(1, 1), Point(-1, 1)),
            Line(Point(-1, 1), Point(-1, -1)),
        ]
        self.state = {}
        self.image_data = []
        self.get_dataset_image = None
        self.boundingbox = []
        self.radius = 0.0
        self.init_boundingbox()

    def from_json(self, config):
        """
        Load a robot from a JSON config dict.
        """
        DEVICES = importlib.import_module("aitk.robots.devices")

        if "name" in config:
            self.name = config["name"]

        if "state" in config:
            self.state = config["state"]

        if "do_trace" in config:
            self.do_trace = config["do_trace"]

        if "va" in config:
            self.va = config["va"]
        if "vx" in config:
            self.vx = config["vx"]
        if "vy" in config:
            self.vy = config["vy"]

        if "tva" in config:
            self.tva = config["tva"]
        if "tvx" in config:
            self.tvx = config["tvx"]
        if "tvy" in config:
            self.tvy = config["tvy"]

        if "va_max" in config:
            self.va_max = config["va_max"]
        if "vx_max" in config:
            self.vx_max = config["vx_max"]
        if "vy_max" in config:
            self.vy_max = config["vy_max"]

        if "va_ramp" in config:
            self.va_ramp = config["va_ramp"]
        if "vx_ramp" in config:
            self.vx_ramp = config["vx_ramp"]
        if "vy_ramp" in config:
            self.vy_ramp = config["vy_ramp"]

        if "x" in config:
            self.x = config["x"]
        if "y" in config:
            self.y = config["y"]
        if "direction" in config:
            self.direction = config["direction"] * PI_OVER_180

        if "image_data" in config:
            self.image_data = config["image_data"]  # ["dataset", index]
        if len(self.image_data) == 0:
            self.get_dataset_image = None
        else:
            self.get_dataset_image = get_dataset(self.image_data[0])

        if "height" in config:
            self.height = config["height"]  # ratio, 0 to 1 of height

        if "color" in config:
            self._set_color(config["color"])

        if "max_trace_length" in config:
            self.max_trace_length = config["max_trace_length"]

        if "body" in config:
            self.body[:] = config["body"]
            self.init_boundingbox()

        if "devices" in config:
            # FIXME: raise if lengths/types don't match
            for i, deviceConfig in enumerate(config["devices"]):
                if i < len(self):
                    if self[i].__class__.__name__ == deviceConfig["class"]:
                        # already a device, let's reuse it:
                        device = self[i]
                        device.initialize()
                        device.from_json(deviceConfig)
                    else:
                        raise Exception(
                            "can't use reset; config changed; use load_world"
                        )
                else:
                    device = None
                    try:
                        device_class = getattr(DEVICES, deviceConfig["class"])
                        device = device_class(**deviceConfig)
                    except Exception:
                        raise Exception(
                            "Failed to create device: %s(**%s)"
                            % (deviceConfig["class"], deviceConfig)
                        )
                    if device:
                        self.add_device(device)

    def info(self):
        """
        Get information on a robot.
        """
        if len(self._devices) == 0:
            print("  This robot has no devices.")
        else:
            for i, device in enumerate(self._devices):
                print(
                    "      robot[%s or %r or %r]: %r"
                    % (i, device.type, device.name, device)
                )
            print("  " + ("-" * 25))

    def plot(
        self, function, x_label="x", y_label="y", title=None,
    ):
        from .plots import Plot

        if title is None:
            title = "%r Robot" % self.name

        plot = Plot(self, function, x_label, y_label, title)
        self.world.watchers.append(plot)
        return plot

    def watch(self, size=100, show_robot=True):
        """
        Watch the robot stats with live updates.

        Args:
            * size: (int) size in pixels around robot
            * show_robot: (bool) show picture of robot
        """
        from .watchers import RobotWatcher

        robot_watcher = RobotWatcher(self, size=size, show_robot=show_robot)
        self.world.watchers.append(robot_watcher)
        # Return the widget:
        return robot_watcher.watch()

    def set_max_trace_length(self, seconds):
        """
        Set the max length of trace, in seconds.

        Args:
            * seconds: (number) the length of trace
        """
        self.max_trace_length = seconds

    def set_color(self, color):
        """
        Set the color of a robot, and its trace.
        """
        self._set_color(color)

    def _set_color(self, color):
        if not isinstance(color, Color):
            self.color = Color(color)
        else:
            self.color = color
        self.trace_color = Color(
            self.color.red * 0.75, self.color.green * 0.75, self.color.blue * 0.75,
        )

    def set_pose(self, x=None, y=None, direction=None, clear_trace=True):
        """
        Set the pose of the robot. direction is in degrees.

        Note: the robot must be in a world.
        """
        if self.world is None:
            raise Exception(
                "This robot is not in a world; add to world before setting pose"
            )
        else:
            if direction is not None:
                direction = (TWO_PI - (direction * PI_OVER_180))
            self._set_pose(x, y, direction, clear_trace)
            # Save the robot's pose to the config
            self.world.update()
            self.world.save()

    def set_random_pose(self, clear_trace=True):
        """
        Set the pose of the robot to a random location.

        Note: the robot must be in a world.
        """
        if self.world is None:
            raise Exception(
                "This robot is not in a world; add robot to world before calling set_random_pose"
            )
        else:
            x, y, direction = self.world._find_random_pose(self)
            self._set_pose(x, y, direction, clear_trace)
            # Save the robot's pose to the config
            self.world.update()
            self.world.save()

    def _set_pose(self, x=None, y=None, direction=None, clear_trace=True):
        """
        Set the pose of the robot. direction is in radians.
        """
        if clear_trace:
            self.trace[:] = []
            self.text_trace[:] = []
            self.pen_trace[:] = []
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if direction is not None:
            self.direction = direction

    def del_device(self, device):
        """
        Remove a device from a robot.
        """
        if isinstance(device, (str, int)):
            device = self[device]
        if device in self._devices:
            device.robot = None
            self._devices.remove(device)
        else:
            raise Exception("Device %r is not on robot." % device)

    def add_device(self, device):
        """
        Add a device to a robot.
        """
        if device not in self._devices:
            device.robot = self
            self._devices.append(device)
            if self.world is not None:
                self.world.update()  # request draw
        else:
            raise Exception("Can't add the same device to a robot more than once.")

    def to_json(self):
        """
        Get this robot as a JSON config file.
        """
        robot_json = {
            "name": self.name,
            "state": self.state,
            "va": self.va,
            "vx": self.vx,
            "vy": self.vy,
            "tva": self.tva,
            "tvx": self.tvx,
            "tvy": self.tvy,
            "va_max": self.va_max,
            "vx_max": self.vx_max,
            "vy_max": self.vy_max,
            "va_ramp": self.va_ramp,
            "vx_ramp": self.vx_ramp,
            "vy_ramp": self.vy_ramp,
            "x": self.x,
            "y": self.y,
            "direction": self.direction * ONE80_OVER_PI,
            "image_data": self.image_data,
            "height": self.height,
            "color": str(self.color),
            "max_trace_length": self.max_trace_length,
            "body": self.body,
            "devices": [device.to_json() for device in self._devices],
            "do_trace": self.do_trace,
        }
        return robot_json

    def move(self, translate=None, rotate=None):
        """
        Set the target translate and rotate velocities.

        Args should be between -1 and 1.
        """
        # values between -1 and 1
        # compute target velocities
        if translate is not None:
            self.tvx = round(translate * self.vx_max, 1)
        if rotate is not None:
            self.tva = round(rotate * self.va_max, 1)

    def forward(self, translate):
        """
        Set the target translate velocity.

        Arg should be between -1 and 1.
        """
        # values between -1 and 1
        self.tvx = round(translate * self.vx_max, 1)

    def backward(self, translate):
        """
        Set the target translate velocity.

        Arg should be between -1 and 1.
        """
        # values between -1 and 1
        self.tvx = round(-translate * self.vx_max, 1)

    def reverse(self):
        """
        Flip the target x velocity from negative to
        positive or from positive to negative.
        """
        self.tvx = -self.tvx

    def turn(self, rotate):
        """
        Set the target rotate velocity.

        Arg should be between -1 and 1.
        """
        # values between -1 and 1
        self.tva = rotate * self.va_max

    def stop(self):
        """
        Set the target velocities to zeros.
        """
        self.tvx = 0.0
        self.tvy = 0.0
        self.tva = 0.0

    def has_image(self):
        """
        Does this robot have an associated 3D set of images from
        a dataset?
        """
        return self.get_dataset_image is not None

    def speak(self, text=None):
        """
        Show some text in the robot's speech bubble.

        Args:
            * text: (str) the text to show; use None to clear

        Note: not for use in a robot in a recorder.
        """
        if self.world:
            if self.world.recording:
                if len(self.text_trace) > 0:
                    # If same as last, don't add again
                    if self.text_trace[-1][1] != text:
                        self.text_trace.append((self.world.time, text))
                else:
                    self.text_trace.append((self.world.time, text))
            else:
                self.text_trace[:] = [(self.world.time, text)]

    def pen_down(self, color, radius=1):
        """
        Put the pen down to change the color of the background image.

        Note: not for use in a robot in a recorder.
        """
        self.pen = (Color(color), radius)

    def pen_up(self):
        """
        Put the pen up to stop changing the color of the background image.

        Note: not for use in a robot in a recorder.
        """
        self.pen = (None, 0)

    def get_max_trace_length(self):
        """
        Get the max step lengths of the trace.
        """
        return self.max_trace_length

    def get_image(self, degrees):
        """
        Return the 3D image in the proper angle.
        """
        return self.get_dataset_image(self.image_data[1], degrees)

    def get_pose(self):
        """
        Get the pose of the robot (x, y, direction) where direction
        is in degrees.
        """
        return (self.x, self.y, ((self.direction + math.pi) * ONE80_OVER_PI) % 360)

    def cast_ray(self, x1, y1, a, maxRange):
        """
        Cast a ray into this world and see what it hits.

        Returns list of hits, furthest away first (back to front)
        """
        # walls and robots
        hits = []
        x2 = math.sin(a) * maxRange + x1
        y2 = math.cos(a) * maxRange + y1

        for wall in self.world.walls:
            # never detect hit with yourself
            if wall.robot is self:
                continue
            for line in wall.lines:
                p1 = line.p1
                p2 = line.p2
                pos = intersect_hit(x1, y1, x2, y2, p1.x, p1.y, p2.x, p2.y)
                if pos is not None:
                    dist = distance(pos[0], pos[1], x1, y1)
                    height = 1.0 if wall.robot is None else wall.robot.height
                    color = wall.robot.color if wall.robot else wall.color
                    boundary = len(wall.lines) == 1
                    hits.append(
                        Hit(
                            wall.robot,
                            height,
                            pos[0],
                            pos[1],
                            dist,
                            color,
                            x1,
                            y1,
                            boundary,
                        )
                    )

        hits.sort(
            key=lambda a: a.distance, reverse=True
        )  # further away first, back to front
        return hits

    def init_boundingbox(self):
        # First, find min/max points around robot (assumes box):
        min_x = float("inf")
        max_x = float("-inf")
        min_y = float("inf")
        max_y = float("-inf")
        max_dist = float("-inf")

        if len(self.body) > 0 and self.body[0][0] == "polygon":
            # "polygon", color, points
            for point in self.body[0][2]:
                min_x = min(min_x, point[0])
                min_y = min(min_y, point[1])
                max_x = max(max_x, point[0])
                max_y = max(max_y, point[1])
                max_dist = max(max_dist, distance(0, 0, point[0], point[1]))

        if (
            min_x == float("inf")
            or min_y == float("inf")
            or max_x == float("-inf")
            or max_y == float("-inf")
        ):
            return

        self.boundingbox = [min_x, min_y, max_x, max_y]
        self.radius = max_dist
        ps = self.compute_boundingbox(self.x, self.y, self.direction)
        self.update_boundingbox(*ps)

    def compute_boundingbox(self, px, py, pdirection):
        # Compute position in real world with respect to x, y, direction:
        min_x, min_y, max_x, max_y = self.boundingbox
        ps = []
        for x, y in [
            (min_x, max_y),  # 4
            (min_x, min_y),  # 1
            (max_x, min_y),  # 2
            (max_x, max_y),  # 3
        ]:
            dist = distance(0, 0, x, y)
            angle = math.atan2(-x, y)
            p = self.rotate_around(px, py, dist, pdirection + angle + PI_OVER_2)
            ps.append(p)
        return ps

    def reset(self):
        """
        Reset the robot's internal stuff. Typeocally, called from the world.
        """
        self.trace[:] = []
        self.text_trace[:] = []
        self.pen_trace[:] = []

    def restore_boundingbox(self):
        self.update_boundingbox(*self.last_boundingbox)

    def update_boundingbox(self, p1, p2, p3, p4):
        self.last_boundingbox = [
            self.bounding_lines[0].p1.copy(),  # p1
            self.bounding_lines[0].p2.copy(),  # p2
            self.bounding_lines[1].p2.copy(),  # p3
            self.bounding_lines[2].p2.copy(),  # p4
        ]
        self.bounding_lines[0].p1.x = p1[0]
        self.bounding_lines[0].p1.y = p1[1]
        self.bounding_lines[0].p2.x = p2[0]
        self.bounding_lines[0].p2.y = p2[1]

        self.bounding_lines[1].p1.x = p2[0]
        self.bounding_lines[1].p1.y = p2[1]
        self.bounding_lines[1].p2.x = p3[0]
        self.bounding_lines[1].p2.y = p3[1]

        self.bounding_lines[2].p1.x = p3[0]
        self.bounding_lines[2].p1.y = p3[1]
        self.bounding_lines[2].p2.x = p4[0]
        self.bounding_lines[2].p2.y = p4[1]

        self.bounding_lines[3].p1.x = p4[0]
        self.bounding_lines[3].p1.y = p4[1]
        self.bounding_lines[3].p2.x = p1[0]
        self.bounding_lines[3].p2.y = p1[1]

    def _deltav(self, tv, v, maxv, ramp, time_step):
        # max change occurs in how long:
        seconds = ramp
        # how much can we change in one time step?
        spt = seconds / time_step
        dv = maxv / spt  # change in one time step
        return min(max(tv - v, -dv), dv)  # keep in limit

    def step(self, time_step):
        """
        Have the robot make one step in time. Check to see if it hits
        any obstacles.
        """
        # proposed acceleration:
        va = self.va + self._deltav(
            self.tva, self.va, self.va_max, self.va_ramp, time_step
        )
        vx = self.vx + self._deltav(
            self.tvx, self.vx, self.vx_max, self.vx_ramp, time_step
        )
        vy = self.vy + self._deltav(
            self.tvy, self.vy, self.vy_max, self.vy_ramp, time_step
        )
        # graphics offset:
        offset = PI_OVER_2
        # proposed positions:
        pdirection = self.direction - va * time_step
        tvx = (
            vx * math.sin(-pdirection + offset)
            + vy * math.cos(-pdirection + offset) * time_step
        )
        tvy = (
            vx * math.cos(-pdirection + offset)
            - vy * math.sin(-pdirection + offset) * time_step
        )
        px = self.x + tvx
        py = self.y + tvy

        # check to see if collision
        # bounding box:
        p1, p2, p3, p4 = self.compute_boundingbox(px, py, pdirection)
        # Set wall bounding boxes for collision detection:
        self.update_boundingbox(p1, p2, p3, p4)

        self.stalled = False
        # if intersection, can't move:
        for wall in self.world.walls:
            if wall.robot is self:  # if yourself, don't check for collision
                continue
            for line in wall.lines:
                w1 = line.p1
                w2 = line.p2
                if (
                    intersect(p1[0], p1[1], p2[0], p2[1], w1.x, w1.y, w2.x, w2.y)
                    or intersect(p2[0], p2[1], p3[0], p3[1], w1.x, w1.y, w2.x, w2.y)
                    or intersect(p3[0], p3[1], p4[0], p4[1], w1.x, w1.y, w2.x, w2.y)
                    or intersect(p4[0], p4[1], p1[0], p1[1], w1.x, w1.y, w2.x, w2.y)
                ):
                    self.stalled = True
                    break

        if not self.stalled:
            # if no intersection, make move
            self.va = va
            self.vx = vx
            self.vy = vy
            self.x = px
            self.y = py
            self.direction = pdirection
        else:
            self.restore_boundingbox()
            # Adjust actual velocity
            self.va = 0
            self.vx = 0
            self.vy = 0

        # Devices:
        for device in self._devices:
            device.step(time_step)

        # Update history:
        if self.do_trace:
            self.trace.append((Point(self.x, self.y), self.direction))

    def update(self, draw_list=None):
        """
        Update the robot, and devices.
        """
        self.init_boundingbox()
        if self.world.debug and draw_list is not None:
            draw_list.append(("strokeStyle", (Color(255), 1)))
            draw_list.append(
                (
                    "draw_line",
                    (
                        self.bounding_lines[0].p1.x,
                        self.bounding_lines[0].p1.y,
                        self.bounding_lines[0].p2.x,
                        self.bounding_lines[0].p2.y,
                    ),
                )
            )

            draw_list.append(
                (
                    "draw_line",
                    (
                        self.bounding_lines[1].p1.x,
                        self.bounding_lines[1].p1.y,
                        self.bounding_lines[1].p2.x,
                        self.bounding_lines[1].p2.y,
                    ),
                )
            )

            draw_list.append(
                (
                    "draw_line",
                    (
                        self.bounding_lines[2].p1.x,
                        self.bounding_lines[2].p1.y,
                        self.bounding_lines[2].p2.x,
                        self.bounding_lines[2].p2.y,
                    ),
                )
            )

            draw_list.append(
                (
                    "draw_line",
                    (
                        self.bounding_lines[3].p1.x,
                        self.bounding_lines[3].p1.y,
                        self.bounding_lines[3].p2.x,
                        self.bounding_lines[3].p2.y,
                    ),
                )
            )

        # Devices:
        for device in self._devices:
            device.update(draw_list)

        # Update recording info:
        if self.world.recording:
            if len(self.pen_trace) > 0:
                # color of pen:
                if self.pen_trace[-1][1][0] != self.pen[0]:
                    self.pen_trace.append((self.world.time, self.pen))
                # same pen color, do nothing
            elif self.pen != (None, 0):
                self.pen_trace.append((self.world.time, self.pen))
            # else do nothing
        else:  # not recording
            if self.pen == (None, 0):
                self.pen_trace[:] = []
            else:
                self.pen_trace[:] = [(self.world.time, self.pen)]

        # Alter world:
        self.update_ground_image(self.world.time)
        return

    def rotate_around(self, x1, y1, length, angle):
        """
        Swing a line around a point.
        """
        return [x1 + length * math.cos(-angle),
                y1 - length * math.sin(-angle)]

    def get_current_text(self, time):
        """
        Get the text for the specific time.
        """
        # FIXME: rewrite as binary search
        if len(self.text_trace) > 0:
            # find the last text that is after time
            for index in range(-1, -len(self.text_trace) - 1, -1):
                curr_time, curr_text = self.text_trace[index]
                if curr_time <= time:
                    return curr_text

    def get_current_pen_color(self, time):
        # FIXME: rewrite as binary search
        if len(self.pen_trace) > 0:
            # find the last color that is after time
            for index in range(-1, -len(self.pen_trace) - 1, -1):
                data = self.pen_trace[index]
                # time, (color, radius)
                if data[0] <= time:
                    return data

    def update_ground_image(self, world_time):
        data = self.get_current_pen_color(world_time)
        # time, (color, radius)
        if data is not None and data[1][0] is not None:
            self.world.set_ground_color_at(self.x, self.y, data[1])

    def draw(self, backend):
        """
        Draw the robot.
        """
        if self.do_trace:
            time_step = self.world.time_step if self.world is not None else 0.1
            max_trace_length = int(1.0 / time_step * self.max_trace_length)

            backend.draw_lines(
                [
                    (point[0], point[1])
                    for (point, direction) in self.trace[-max_trace_length:]
                ],
                stroke_style=self.trace_color,
            )
            self.trace = self.trace[-max_trace_length:]

        backend.pushMatrix()
        backend.translate(self.x, self.y)
        backend.rotate(self.direction)

        # body:

        for shape in self.body:
            shape_name, color, args = shape

            if self.stalled:
                backend.set_fill(Color(128, 128, 128))
                backend.strokeStyle(Color(255), 1)
            elif color is None:
                backend.set_fill(self.color)
                backend.noStroke()
            else:
                backend.set_fill(Color(color))
                backend.noStroke()

            if shape_name == "polygon":
                backend.draw_polygon(args)
            elif shape_name == "rectangle":
                backend.draw_rect(*args)
            elif shape_name == "ellipse":
                backend.draw_ellipse(*args)
            elif shape_name == "circle":
                backend.draw_circle(*args)

        for device in self._devices:
            device.draw(backend)

        backend.popMatrix()

        text = self.get_current_text(self.world.time)
        if text:
            backend.set_fill_style(Color(255))
            pad = 10
            box_pad = 5
            width = self.world.backend.char_width * len(text)
            height = 20
            if self.x - pad - width < 0:
                side = 1  # put on right
            else:
                side = -1  # put on left
            if self.y - height < 0:
                half = 1  # put on top
            else:
                half = -1  # put on bottom
            points = [
                (self.x, self.y),
                (self.x + pad * side, self.y + height / 2 * half),
                (self.x + pad * side, self.y + height * half),
                (self.x + (pad + width + pad) * side, self.y + height * half),
                (self.x + (pad + width + pad) * side, self.y),
                (self.x + pad * side, self.y),
                (self.x + pad * side, self.y + height / 4 * half),
            ]
            backend.set_stroke_style(Color(0))
            backend.set_fill_style(Color(255, 255, 255, 200))
            backend.draw_polygon(points)
            backend.set_fill_style(Color(0))
            if side == 1:  # right
                if half == 1:  # bottom
                    backend.text(text, self.x + (pad + box_pad), self.y + box_pad)
                else:  # top
                    backend.text(
                        text,
                        self.x + (pad + box_pad),
                        self.y - self.world.backend.char_height - box_pad,
                    )
            else:  # left
                if half == 1:  # bottom
                    backend.text(text, self.x - pad - width - box_pad, self.y + box_pad)
                else:  # top
                    backend.text(
                        text,
                        self.x - pad - width - box_pad,
                        self.y - self.world.backend.char_height - box_pad,
                    )


SCRIBBLER_CONFIG = {
    "body": [
        [
            "polygon",
            None,
            [
                [4.17, 5],
                [4.17, 6.67],
                [5.83, 5.83],
                [5.83, 5],
                [7.5, 5],
                [7.5, -5],
                [5.83, -5],
                [5.83, -5.83],
                [4.17, -6.67],
                [4.17, -5],
                [-4.17, -5],
                [-4.17, -6.67],
                [-5.83, -5.83],
                [-6.67, -5],
                [-7.5, -4.17],
                [-7.5, 4.17],
                [-6.67, 5],
                [-5.83, 5.83],
                [-4.17, 6.67],
                [-4.17, 5],
            ],
        ],
        ["rectangle", "black", [-3.33, -7.67, 6.33, 1.67]],
        ["rectangle", "black", [-3.33, 6, 6.33, 1.67]],
        ["polygon", "black", [[0, 0], [-2, 2], [2, 0], [-2, -2]]],
    ],
}


class Scribbler(Robot):
    def __init__(
        self,
        x=0,
        y=0,
        direction=0,
        color="red",
        name="Scribbie",
        do_trace=True,
        height=0.25,
        max_trace_length=10,
        **kwargs
    ):
        """
        A small little two-wheeled robot. x,y should fit in the world that
        you will place the robot into (or use x=0, y=0 to put in a random place).

        Args:
            * x: (int) starting location in the horizontal direction. Leave 0 to
                place in a random location.
            * y: (int) starting location in the horizontal direction. Leave 0 to
                place in a random location.
            * direction: (number) starting angle in degrees.
            * color:
            * name: (str) a name to give your robot
            * do_trace: (bool) should the robot leave a trace?
            * height: (number) height of robot (use number < 1)
            * max_trace_length: (number) max length of trace, in seconds

        Any of the other valid config settings can also be passed in, including:
            * state: (dict) serializable memory for a robot
            * va, vx, vy: (numbers) velocities
            * tva, tvx, tvy: (numbers) target velocities
            * va_max, vx_max, vy_max: (numbers) max velocities
            * va_ramp, vx_ramp, vy_ramp: (numbers) linear accelerations
            * image_data: ["dataset-name", index] to use a 3D set of images
            * body: data structure that defines a robot body
            * devices: list of serialized devices
        """
        config = {
            "x": x,
            "y": y,
            "direction": direction,
            "color": color,
            "name": name,
            "do_trace": do_trace,
            "height": height,
            "max_trace_length": max_trace_length,
        }
        for item in [
            "state",
            "va",
            "vx",
            "vy",
            "tva",
            "tvx",
            "tvy",
            "va_max",
            "vx_max",
            "vy_max",
            "va_ramp",
            "vx_ramp",
            "vy_ramp",
            "image_data",
            "body",
            "devices",
        ]:
            arg = kwargs.pop(item, None)
            if arg is not None:
                config[item] = arg
        defaults = SCRIBBLER_CONFIG.copy()
        defaults.update(config)
        super().__init__(**defaults)
