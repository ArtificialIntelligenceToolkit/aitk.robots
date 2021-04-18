# -*- coding: utf-8 -*-
# ************************************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2021 AITK Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
# ************************************************************

import importlib
import math
import re

from .datasets import get_dataset
from .hit import Hit
from .utils import (
    arange,
    display,
    Color,
    Line,
    Point,
    degrees_to_world,
    distance,
    intersect,
    intersect_hit,
    rotate_around,
    PI_OVER_180,
    PI_OVER_2,
    ONE80_OVER_PI,
    TWO_PI,
    world_to_degrees,
)

class Robot:
    """
    The base robot class.
    """

    def __init__(
        self,
        x=0,
        y=0,
        a=0,
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
        # Get the args:
        config = {
            "x": x,
            "y": y,
            "a": a, # degrees in the config file
            "color": color,
            "name": name,
            "do_trace": do_trace,
            "height": height,
            "max_trace_length": max_trace_length,
        }
        # Update from the kwargs:
        config.update(kwargs)
        self.world = None
        self._devices = []
        self._initialize()
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
            return "<Robot(name=%r, x=%s, y=%s, a=%s)>" % (
                self.name,
                round(self.x, 2),
                round(self.y, 2),
                round(world_to_degrees(self.a), 2),
            )

    def _initialize(self):
        """
        Initialize the robot properties.
        """
        self.name = "Robbie"
        self.state = {}
        self._set_color("red")
        self.eat_food_distance = 20
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
        self.a = degrees_to_world(0)  # radians
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
        self._bounding_lines = [
            Line(Point(-1, -1), Point(1, -1)),
            Line(Point(1, -1), Point(1, 1)),
            Line(Point(1, 1), Point(-1, 1)),
            Line(Point(-1, 1), Point(-1, -1)),
        ]
        self.state = {}
        self.image_data = []
        self.get_dataset_image = None
        self._boundingbox = []
        self.radius = 0.0
        self.food_eaten = 0
        self._watcher = None
        self._init_boundingbox()

    def from_json(self, config):
        """
        Load a robot from a JSON config dict.
        """
        DEVICES = importlib.import_module("aitk.robots.devices")
        valid_keys = set([
            "name", "state", "do_trace", "va", "vx", "vy",
            "tva", "tvx", "tvy", "x", "y", "a", "va_max",
            "vx_max", "vy_max", "va_ramp", "vx_ramp", "vy_ramp",
            "image_data", "height", "color", "max_trace_length",
            "body", "devices",
        ])
        config_keys = set(list(config.keys()))
        extra_keys = config_keys - valid_keys

        if len(extra_keys) > 0:
            raise TypeError("invalid key(s) for robot config: %r" % extra_keys)

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
        if "a" in config:
            self.a = degrees_to_world(config["a"])

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
            self._init_boundingbox()

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

    def summary(self):
        """
        Get a summary of information about the robot.
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

    def get_widget(self, size=None, show_robot=None, attributes=None):
        """
        Get the robot widget.

        Args:
            * size: (int) size in pixels around robot
            * show_robot: (bool) show picture of robot
            * attributes: (list) items to include, or "all"
        """
        from .watchers import RobotWatcher

        if self._watcher is None:
            size = size if size is not None else 100
            show_robot = show_robot if show_robot is not None else True
            attributes = attributes if attributes is not None else "all"
            self._watcher = RobotWatcher(self, size=size,
                                         show_robot=show_robot,
                                         attributes=attributes)
            self.world._watchers.append(self._watcher)
        else:
            self._watcher.set_arguments(size=size,
                                        show_robot=show_robot,
                                        attributes=attributes)
            self._watcher.draw()

        return self._watcher.get_widget()

    def watch(self, size=None, show_robot=None, attributes=None):
        """
        Watch the robot stats with live updates.

        Args:
            * size: (int) size in pixels around robot
            * show_robot: (bool) show picture of robot
            * attributes: (list) items to include, or "all"
        """
        widget = self.get_widget(
            size=size,
            show_robot=show_robot,
            attributes=attributes,
        )
        display(widget)

    def get_image(self, size=100):
        """
        Get an image of the robot.

        Args:
            * size: (int) size in pixels around robot
        """
        picture = self.world.get_image()
        start_x = round(
            max(self.x * self.world.scale - size / 2, 0)
        )
        start_y = round(
            max(self.y * self.world.scale - size / 2, 0)
        )
        rectangle = (
            start_x,
            start_y,
            min(
                start_x + size, self.world.width * self.world.scale
            ),
            min(
                start_y + size,
                self.world.height * self.world.scale,
            ),
        )
        picture = picture.crop(rectangle)
        return picture

    def display(self, size=100):
        """
        Display the robot's image.
        """
        image = self.get_image(size=size)
        display(image)

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

    def set_pose(self, x=None, y=None, a=None, clear_trace=True):
        """
        Set the pose of the robot. a is in degrees.

        Note: the robot must be in a world.
        """
        if self.world is None:
            raise Exception(
                "This robot is not in a world; add to world before setting pose"
            )
        else:
            if a is not None:
                a = degrees_to_world(a)
            self._set_pose(x, y, a, clear_trace)
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
            # Direction is in radians, in world coordinates:
            x, y, a = self.world._find_random_pose(self)
            self._set_pose(x, y, a, clear_trace)
            # Save the robot's pose to the config
            self.world.update()
            self.world.save()

    def _set_pose(self, x=None, y=None, a=None, clear_trace=True):
        """
        Set the pose of the robot. direction (a) is in radians.

        a: direction is in radians, in world coordinates.
        """
        if clear_trace:
            self.trace[:] = []
            self.text_trace[:] = []
            self.pen_trace[:] = []
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if a is not None:
            self.a = a

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

    def add_device_ring(self, device_class, distance_from_center,
                        start_degree, stop_degree, count, **kwargs):
        """
        Adds a ring of devices at a given distance from the center of
        the robot.

        Args:
            device_class: a class or function that receives position,
                a, and kwargs, and returns a device
            distance_from_center: in CM
            start_degree: angle of first device (0 points right)
            stop_degree: angle of stop degree (counter clockwise)
            count: number of sensors to add
            kwargs: additional args to pass to device_class

        Example:

        ```python
        >>> robot.add_device_ring(RangeSensor, 10, 0, 359, 6, width=20)
        ```
        """
        span = stop_degree - start_degree
        step_angle = span / count
        for angle in arange(start_degree, stop_degree, step_angle):
            x, y = rotate_around(0, 0, 7, -angle * PI_OVER_180)
            self.add_device(device_class(position=(x, y), a=angle, **kwargs))

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
            "a": world_to_degrees(self.a),
            "image_data": self.image_data,
            "height": self.height,
            "color": str(self.color),
            "max_trace_length": self.max_trace_length,
            "body": self.body,
            "devices": [device.to_json() for device in self._devices],
            "do_trace": self.do_trace,
        }
        return robot_json

    def motors(self, left, right):
        """
        A move function that takes desired motor values
        and converts to trans and rotate.
        """
        trans = (right + left) / 2.0
        rotate = (right - left) / 2.0
        self.move(trans, rotate)

    def move(self, translate, rotate):
        """
        Set the target translate and rotate velocities.

        Args should be between -1 and 1.
        """
        # values between -1 and 1
        # compute target velocities
        if self.world is not None:
            if self.world.status != "running":
                print("This world is not running")
        if translate is not None:
            self.tvx = round(translate * self.vx_max, 1)
        if rotate is not None:
            self.tva = round(rotate * self.va_max, 1)

    def translate(self, translate):
        """
        Set the target translate velocity.

        Arg should be between -1 and 1.
        """
        # values between -1 and 1
        self.tvx = round(translate * self.vx_max, 1)

    def forward(self, translate):
        """
        Set the target translate velocity.

        Arg should be between 0 and 1, inclusive.
        """
        if 0 <= translate <= 1:
            self.tvx = round(translate * self.vx_max, 1)
        else:
            print("forward value is out of range; should be between 0 and 1, inclusive")

    def backward(self, translate):
        """
        Set the target translate velocity.

        translate should be between 0 and 1, inclusive.
        """
        if 0 <= translate <= 1:
            self.tvx = round(-translate * self.vx_max, 1)
        else:
            print("backward value is out of range; should be between 0 and 1, inclusive")

    def reverse(self):
        """
        Flip the target x and a velocities from negative to
        positive or from positive to negative.
        """
        self.tvx = -self.tvx
        self.tva = -self.tva

    def rotate(self, rotate):
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

    def eat(self):
        """
        If the robot is close enough to food, then this
        will eat it, removing from the world, and requesting
        a redraw. Returns True if success, and False
        otherwise.

        Note: it must be within robot.eat_food_distance
        """
        success = False
        if self.world is not None:
            for food in self.world._food[:]: # copy
                if distance(self.x, self.y, food[0], food[1]) <= self.eat_food_distance:
                    self.food_eaten += 1
                    success = True
                    self.world._food.remove(food)
                    self.world._grid.need_update = True
                    self.world.update() # request draw
        return success

    def speak(self, text=None):
        """
        Show some text in the robot's speech bubble.

        Args:
            * text: (str) the text to show; use None to clear

        Note: not for use in a robot in a recorder.
        """
        if self.world:
            if self.world._recording:
                if len(self.text_trace) > 0:
                    # If same as last, don't add again
                    if self.text_trace[-1][1] != text:
                        self.text_trace.append((self.world.time, text))
                else:
                    self.text_trace.append((self.world.time, text))
            else:
                self.text_trace[:] = [(self.world.time, text)]

    def pen_down(self, color=None, radius=1):
        """
        Put the pen down to change the color of the background image.

        Note: not for use in a robot in a recorder.
        """
        from PIL import Image

        color = color if color is not None else self.color
        self.pen = (Color(color), radius)
        if self.world is not None:
            if self.world._ground_image is None:
                image = Image.new("RGBA", (self.world.width, self.world.height),
                                  color="white")
                filename = "ground_image.png"
                image.save(filename)
                self.world.set_ground_image(filename)

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

    def get_image_3d(self, degrees):
        """
        Return the 3D image in the proper angle.
        """
        return self.get_dataset_image(self.image_data[1], degrees)

    def get_time(self):
        """
        Get the clock time of the world.
        """
        if self.world:
            return self.world.time

    def get_pose(self):
        """
        Get the pose of the robot (x, y, a) where a (direction)
        is in degrees.
        """
        return (self.x, self.y, world_to_degrees(self.a))

    def get_velocity(self, target=False):
        """
        Get the current (or target) translate and rotate velocities
        of the robot.
        """
        if not target:
            return (self.vx / self.vx_max, self.va / self.va_max)
        else:
            return (self.tvx / self.vx_max, self.tva / self.va_max)

    def cast_ray(self, x1, y1, a, maxRange, x2=None, y2=None, ignore_robots=None):
        """
        Cast a ray into this world and see what it hits.

        Returns list of hits, furthest away first (back to front)
        """
        # walls and robots
        hits = []
        if x2 is None:
            x2 = math.sin(a) * maxRange + x1
        if y2 is None:
            y2 = math.cos(a) * maxRange + y1

        for wall in self.world._walls:
            # never detect hit with yourself
            if wall.robot is self:
                continue
            # ignore this robot:
            if ((ignore_robots is not None) and
                (wall.robot is not None) and
                (wall.robot in ignore_robots)):
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
                            a,
                        )
                    )

        hits.sort(
            key=lambda a: a.distance, reverse=True
        )  # further away first, back to front
        return hits

    def _init_boundingbox(self):
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

        self._boundingbox = [min_x, min_y, max_x, max_y]
        self.radius = max_dist
        ps = self._compute_boundingbox(self.x, self.y, self.a)
        self._update_boundingbox(*ps)

    def _compute_boundingbox(self, px, py, pa):
        # Compute position in real world with respect to x, y, a:
        if len(self._boundingbox) == 4:
            min_x, min_y, max_x, max_y = self._boundingbox
            ps = []
            for x, y in [
                (min_x, max_y),  # 4
                (min_x, min_y),  # 1
                (max_x, min_y),  # 2
                (max_x, max_y),  # 3
            ]:
                dist = distance(0, 0, x, y)
                angle = math.atan2(-x, y)
                p = rotate_around(px, py, dist, pa + angle + PI_OVER_2)
                ps.append(p)
            return ps
        else:
            raise ValueError("robot %r does not have a body" % self.name)

    def reset(self):
        """
        Reset the robot's internal stuff. Typeocally, called from the world.
        """
        self.trace[:] = []
        self.text_trace[:] = []
        self.pen_trace[:] = []

    def _restore_boundingbox(self):
        self._update_boundingbox(*self._last_boundingbox)

    def _update_boundingbox(self, p1, p2, p3, p4):
        self._last_boundingbox = [
            self._bounding_lines[0].p1.copy(),  # p1
            self._bounding_lines[0].p2.copy(),  # p2
            self._bounding_lines[1].p2.copy(),  # p3
            self._bounding_lines[2].p2.copy(),  # p4
        ]
        self._bounding_lines[0].p1.x = p1[0]
        self._bounding_lines[0].p1.y = p1[1]
        self._bounding_lines[0].p2.x = p2[0]
        self._bounding_lines[0].p2.y = p2[1]

        self._bounding_lines[1].p1.x = p2[0]
        self._bounding_lines[1].p1.y = p2[1]
        self._bounding_lines[1].p2.x = p3[0]
        self._bounding_lines[1].p2.y = p3[1]

        self._bounding_lines[2].p1.x = p3[0]
        self._bounding_lines[2].p1.y = p3[1]
        self._bounding_lines[2].p2.x = p4[0]
        self._bounding_lines[2].p2.y = p4[1]

        self._bounding_lines[3].p1.x = p4[0]
        self._bounding_lines[3].p1.y = p4[1]
        self._bounding_lines[3].p2.x = p1[0]
        self._bounding_lines[3].p2.y = p1[1]

    def _deltav(self, tv, v, maxv, ramp, time_step):
        # max change occurs in how long:
        seconds = ramp
        # how much can we change in one time step?
        spt = seconds / time_step
        dv = maxv / spt  # change in one time step
        return min(max(tv - v, -dv), dv)  # keep in limit

    def _step(self, time_step):
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
        pa = self.a - va * time_step
        tvx = (
            vx * math.sin(-pa + offset)
            + vy * math.cos(-pa + offset) * time_step
        )
        tvy = (
            vx * math.cos(-pa + offset)
            - vy * math.sin(-pa + offset) * time_step
        )
        px = self.x + tvx
        py = self.y + tvy

        # check to see if collision
        # bounding box:
        p1, p2, p3, p4 = self._compute_boundingbox(px, py, pa)
        # Set wall bounding boxes for collision detection:
        self._update_boundingbox(p1, p2, p3, p4)

        self.stalled = False
        # if intersection, can't move:
        for wall in self.world._walls:
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
            self.a = pa
        else:
            self._restore_boundingbox()
            # Adjust actual velocity
            self.va = 0
            self.vx = 0
            self.vy = 0

        # Devices:
        for device in self._devices:
            device._step(time_step)

        # Update history:
        if self.do_trace:
            self.trace.append((Point(self.x, self.y), self.a))

    def update(self, draw_list=None):
        """
        Update the robot, and devices.
        """
        # Wrapped worlds:
        wrapped = False
        if self.x < 0:
            self.x = self.world.width
            wrapped = True
        elif self.x > self.world.width:
            self.x = 0
            wrapped = True
        if self.y < 0:
            self.y = self.world.height
            wrapped = True
        elif self.y > self.world.height:
            self.y = 0
            wrapped = True

        if wrapped:
            self.trace.append(None)

        self._init_boundingbox()

        if self.world.debug and draw_list is not None:
            draw_list.append(("strokeStyle", (Color(255), 1)))
            draw_list.append(
                (
                    "draw_line",
                    (
                        self._bounding_lines[0].p1.x,
                        self._bounding_lines[0].p1.y,
                        self._bounding_lines[0].p2.x,
                        self._bounding_lines[0].p2.y,
                    ),
                )
            )

            draw_list.append(
                (
                    "draw_line",
                    (
                        self._bounding_lines[1].p1.x,
                        self._bounding_lines[1].p1.y,
                        self._bounding_lines[1].p2.x,
                        self._bounding_lines[1].p2.y,
                    ),
                )
            )

            draw_list.append(
                (
                    "draw_line",
                    (
                        self._bounding_lines[2].p1.x,
                        self._bounding_lines[2].p1.y,
                        self._bounding_lines[2].p2.x,
                        self._bounding_lines[2].p2.y,
                    ),
                )
            )

            draw_list.append(
                (
                    "draw_line",
                    (
                        self._bounding_lines[3].p1.x,
                        self._bounding_lines[3].p1.y,
                        self._bounding_lines[3].p2.x,
                        self._bounding_lines[3].p2.y,
                    ),
                )
            )

        # Devices:
        for device in self._devices:
            device.update(draw_list)

        # Update recording info:
        if self.world._recording:
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
        self._update_ground_image(self.world.time)
        return

    def _get_current_text(self, time):
        """
        Get the text for the specific time.
        """
        # FIXME: rewrite as binary search
        if len(self.text_trace) > 0:
            # find the last text that is after time
            if len(self.text_trace) == 1:
                return self.text_trace[0][1]
            for index in range(-1, -len(self.text_trace) - 1, -1):
                curr_time, curr_text = self.text_trace[index]
                if curr_time <= time:
                    return curr_text

    def _get_current_pen_color(self, time):
        # FIXME: rewrite as binary search
        if len(self.pen_trace) > 0:
            # find the last color that is after time
            if len(self.pen_trace) == 1:
                return self.pen_trace[0]
            for index in range(-1, -len(self.pen_trace) - 1, -1):
                data = self.pen_trace[index]
                # time, (color, radius)
                if data[0] <= time:
                    return data

    def _update_ground_image(self, world_time):
        data = self._get_current_pen_color(world_time)
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

            data = self.trace[-max_trace_length:]

            # None indicates a segment break
            if all(data): # no segments
                segments = [[(point[0], point[1]) for (point, direction) in data]]
            else:
                segments = []
                current = []
                for item in data:
                    if item is None:
                        segments.append(current)
                        current = []
                    else:
                        point, direction = item
                        current.append((point[0], point[1]))
                if current:
                    segments.append(current)

            for segment in segments:
                backend.draw_lines(
                    segment,
                    stroke_style=self.trace_color,
                )
            self.trace = self.trace[-max_trace_length:]

        backend.pushMatrix()
        backend.translate(self.x, self.y)
        backend.rotate(self.a)

        # Draw first:
        for device in self._devices:
            if device.type == "bulb":
                device.draw(backend)

        # body:
        for shape in self.body:
            shape_name, color, args = shape

            if self.stalled:
                backend.strokeStyle(Color(255), 1)
            else:
                backend.strokeStyle(Color(0), 1)

            if color is None:
                backend.set_fill(self.color)
            else:
                backend.set_fill(Color(color))

            if shape_name == "polygon":
                backend.draw_polygon(args)
            elif shape_name == "rectangle":
                backend.draw_rect(*args)
            elif shape_name == "ellipse":
                backend.draw_ellipse(*args)
            elif shape_name == "circle":
                backend.draw_circle(*args)
            elif shape_name == "line":
                backend.draw_line(*args)

            backend.noStroke()

        # Draw on top of robot:
        for device in self._devices:
            if device.type != "bulb":
                device.draw(backend)

        backend.popMatrix()

        text = self._get_current_text(self.world.time)
        if text:
            backend.set_fill_style(Color(255))
            pad = 10
            box_pad = 5
            width = self.world._backend.char_width * len(text)
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
                        self.y - self.world._backend.char_height - box_pad,
                    )
            else:  # left
                if half == 1:  # bottom
                    backend.text(text, self.x - pad - width - box_pad, self.y + box_pad)
                else:  # top
                    backend.text(
                        text,
                        self.x - pad - width - box_pad,
                        self.y - self.world._backend.char_height - box_pad,
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
        a=0,
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
            * a: (number) starting angle in degrees.
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
            "a": a, # degrees in config file
            "color": color,
            "name": name,
            "do_trace": do_trace,
            "height": height,
            "max_trace_length": max_trace_length,
        }
        # First, get Scribbler defaults:
        defaults = SCRIBBLER_CONFIG.copy()
        # Then update from args:
        defaults.update(config)
        # Then update from kwargs:
        defaults.update(kwargs)
        super().__init__(**defaults)


VEHICLE_CONFIG = {
    "body": [
        [
            "polygon",
            None,
            [
                [5, 3.25],
                [5, -3.25],
                [-5, -3.25],
                [-5, 3.25],
            ],
        ],
        ["rectangle", "black", [-6, -3.25 - 1.5, 2.75, 1.5]],
        ["rectangle", "black", [-6, 3.25, 2.75, 1.5]],
        ["line", "black", [5, 3, 8, 3]],
        ["line", "black", [5,-3, 8,-3]],
    ],
}

class Vehicle(Robot):
    def __init__(
        self,
        x=0,
        y=0,
        a=0,
        color="white",
        name="Vickie",
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
            * a: (number) starting angle in degrees.
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
            "a": a, # degrees in config file
            "color": color,
            "name": name,
            "do_trace": do_trace,
            "height": height,
            "max_trace_length": max_trace_length,
        }
        # First, get Scribbler defaults:
        defaults = VEHICLE_CONFIG.copy()
        # Then update from args:
        defaults.update(config)
        # Then update from kwargs:
        defaults.update(kwargs)
        super().__init__(**defaults)
