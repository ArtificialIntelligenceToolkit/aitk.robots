# -*- coding: utf-8 -*-
# ************************************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2021 AITK Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
# ************************************************************

import math
import os
import random
import re
import signal
import time
from threading import Thread
from collections.abc import Sequence
from contextlib import contextmanager
from itertools import count
from numbers import Number

from .backends import make_backend
from .devices import Bulb
from .robot import Robot
from .utils import (
    Color,
    Grid,
    Line,
    Point,
    distance,
    distance_point_to_line,
    format_time,
    json_dump,
    load_image,
    progress_bar,
    PI_OVER_180,
    PI_OVER_2,
    ONE80_OVER_PI,
)

DEFAULT_HANDLER = signal.getsignal(signal.SIGINT)


class Wall:
    """
    Class representing obstacles in the world. If the bounding box of
    a robot, then robot will be that robot, else None.
    """

    def __init__(self, color, robot, *lines, wtype=None):
        """
        Create a wall consisting of one or more lines.

        wtype is "boundary", "robot", or "wall"
        """
        self.color = color
        self.robot = robot
        self.lines = lines
        self.wtype = wtype

    def __repr__(self):
        return "Wall(%r, %r, %r, wtype=%r)" % (self.color, self.robot, self.lines, self.wtype)


class List(Sequence):
    def __init__(self, list_object):
        self.list_object = list_object

    def __getattr__(self, attr):
        return getattr(self.list_object, attr)

    def __getitem__(self, item):
        if isinstance(item, int):
            return self.list_object[item]
        elif isinstance(item, str):
            name_map = {}  # mapping of base to count
            search_groups = re.match(r"(.*)-(\d*)", item)
            if search_groups:
                search_name = search_groups[1].lower()
                if search_groups[2].isdigit():
                    search_index = int(search_groups[2])
                else:
                    search_name = item
                    search_index = 1
            else:
                search_name = item.lower()
                search_index = 1
            for item in self.list_object:
                # update name_map
                name = item.name.lower()
                index = None
                if "-" in name:
                    prefix, index = name.rsplit("-", 1)
                    if index.isdigit():
                        name = prefix
                        index = int(index)
                    else:
                        index = 1
                if name not in name_map:
                    name_map[name] = 1
                else:
                    name_map[name] += 1
                if index is None:
                    index = name_map[name]
                if search_name == name and search_index == index:
                    return item

        return None

    def __len__(self):
        return len(self.list_object)

    def __repr__(self):
        return repr(self.list_object)

class CanvasCall:
    def __init__(self, canvas, command):
        self.canvas = canvas
        self.command = command

    def __call__(self, *args, **kwargs):
        self.canvas.command_list.append((self.command, args, kwargs))

class Canvas:
    def __init__(self, command_list):
        self.command_list = command_list

    def __getattr__(self, attr):
        return CanvasCall(self, attr)

    def clear(self):
        self.command_list.clear()

class World:
    """
    The aitk.robots simulator world.
    """

    def __init__(
        self,
        width=500,
        height=250,
        seed=0,
        scale=3.0,
        boundary_wall=True,
        boundary_wall_color="purple",
        boundary_wall_width=1,
        ground_color="green",
        ground_image_filename=None,
        filename=None,
        quiet=False,
        smell_cell_size=None,
        **kwargs
    ):
        """
        The aitk.robots simulator world.

        Args:
            * width: (int) width of world in pixels
            * height: (int) height of world in pixels
            * seed: (int) random number generator seed
            * scale: (number) value to use in drawing the world
            * boundary_wall: (bool) draw a boundary around the world?
            * boundary_wall_color: (str) color name of boundary wall
            * boundary_wall_width: (int) width of boundary wall
            * ground_color: (str) color name
            * ground_image_filename: (str) image file used for backgound
            * filename: (str) name of json world file
            * quiet: (bool) if True, don't print any messages

        You can also pass any valid item from the world config settings.
        """
        # For faster-than real time display with synchronous backends,
        # keep processing time below this percentage of throttle_period:
        config = {
            "width": width,
            "height": height,
            "seed": seed,
            "scale": scale,
            "boundary_wall": boundary_wall,
            "boundary_wall_width": boundary_wall_width,
            "boundary_wall_color": boundary_wall_color,
            "ground_color": ground_color,
            "quiet": quiet,
            "smell_cell_size": smell_cell_size,
        }
        self._messages = []
        if filename is not None:
            config["filename"] = filename
        if ground_image_filename is not None:
            config["ground_image_filename"] = ground_image_filename
        config["walls"] = kwargs.pop("walls", [])
        config["bulbs"] = kwargs.pop("bulbs", [])
        config["robots"] = kwargs.pop("robots", [])
        config["food"] = kwargs.pop("food", [])
        if len(kwargs) != 0:
            raise AttributeError(
                "unknown arguments for World: %s" % list(kwargs.keys())
            )
        self._show_throttle_percentage = 0.40
        self._time_decimal_places = 1
        self._throttle_period = 0.1
        self._time_of_last_call = 0
        self._step_display = "tqdm"
        self.debug = False
        self._watchers = []
        self._robots = []
        self._bulbs = []
        self._backend = None
        self._recording = False
        self.config = config.copy()
        self._initialize()  # default values
        self.robots = List(self._robots)
        self.bulbs = List(self._bulbs)
        self.reset()  # from config

    def __repr__(self):
        return "<World width=%r, height=%r>" % (self.width, self.height)

    def get_image(self, index=None, size=100):
        """
        Get a PIL image of the world, or of a robot.

        Args:
            index: (str or int, optional) - index of robot
            size: (int, optional) - size of robot picture
        """
        try:
            picture = self._backend.get_image(self.time)
        except RuntimeError:
            raise Exception("Backend is not ready yet; try again")

        if index is not None:
            robot = self.robots[index]
            if robot:
                start_x = round(max(robot.x * self.scale - size / 2, 0))
                start_y = round(max(robot.y * self.scale - size / 2, 0))
                rectangle = (
                    start_x,
                    start_y,
                    min(start_x + size, self.width * self.scale),
                    min(start_y + size, self.height * self.scale),
                )
                picture = picture.crop(rectangle)
                return picture
        else:
            return picture

    def display(self, index=None, size=100):
        """
        Display a picture of the world, or of a robot.

        Args:
            index: (str or int, optional) - index of robot
            size: (int, optional) - size of robot picture
        """
        picture = self.get_image(index=index, size=size)
        display(picture)

    def summary(self):
        """
        Get a summary of information about the world and
        all of its robots.
        """
        print("World details:")
        if self.filename:
            print("This world was loaded from %r" % self.filename)
        print("Size: %s x %s" % (self.width, self.height))
        print("Robots:")
        if len(self._robots) == 0:
            print("  This world has no robots.")
        else:
            print("-" * 25)
            for i, robot in enumerate(self._robots):
                print("  .robots[%s or %r]: %r" % (i, robot.name, robot))
                robot.summary()
        print("Food:")
        if len(self._food) == 0:
            print("  This world has no food.")
        else:
            print("-" * 25)
            for food in self._food:
                print("  x: %s, y: %s, brightness: %s" % (food[0], food[1], food[2]))
        print("Lights:")
        if len(self._bulbs) == 0:
            print("  This world has no lights.")
        else:
            print("-" * 25)
            for bulb in self._bulbs:
                print("  x: %s, y: %s, brightness: %s, name: %r, color: %s" % (
                    bulb.x,
                    bulb.y,
                    bulb.brightness,
                    bulb.name,
                    bulb.color))

    def get_robot(self, item):
        """
        Get the robot by name or index. Equivalent to
        world.robots[item]

        Args:
            * item: (int or string) index or name of robot
        """
        return self.robots[item]

    def get_bulb(self, item):
        """
        Get the bulb by name or index. Equivalent to
        world.bulbs[item]

        Args:
            * item: (int or string) index or name of bulb
        """
        return self.bulbs[item]

    def _initialize(self):
        """
        Sets the default values.
        """
        self._draw_list = []
        self._overlay_list = []
        self.canvas = Canvas(self._overlay_list)
        self.filename = None
        self.quiet = False
        self.seed = 0
        self.width = 500
        self.height = 250
        self.scale = 3.0
        self._stop = False  # should stop?
        self._thread = None
        self.status = "stopped"
        self.time_step = 0.10  # seconds
        self.time = 0.0  # seconds
        self.boundary_wall = True
        self.boundary_wall_width = 1
        self.boundary_wall_color = Color(128, 0, 128)
        self.ground_color = Color(0, 128, 0)
        self.ground_image_filename = None
        self._ground_image = None
        self._ground_image_pixels = None
        self._walls = []
        self._bulbs.clear()
        self._complexity = 0
        self.smell_cell_size = None

    def reset(self):
        """
        Reloads the config from initialization, or from
        last save.
        """
        self.stop()
        self._initialize()
        self._reset_watchers()
        self._food = []
        self._grid = None
        self.from_json(self.config)
        self.time = 0.0
        for robot in self._robots:
            robot.reset()
            # Re-add the robot's boundaries:
            wall = Wall(robot.color, robot, *robot._bounding_lines, wtype="robot")
            self._walls.append(wall)
        self._stop = False  # should stop?
        self.status = "stopped"
        self.update(show=False)  # twice to allow robots to see each other
        self.update(show=False)
        self.draw()  # force

    def set_seed(self, seed):
        """
        Set the random seed.
        """
        if seed == 0:
            seed = random.randint(0, 9999999)
            if not self.quiet:
                print("Random seed set to:", seed)
        else:
            if not self.quiet:
                print("Using random seed:", seed)
        random.seed(seed)
        self.seed = seed
        self.config["seed"] = seed

    def from_json(self, config):
        """
        Load a json config file.
        """
        self.config = config
        seed = config.get("seed", 0)
        self.set_seed(seed)

        if "filename" in config:
            self.filename = config["filename"]
        if "quiet" in config:
            self.quiet = config["quiet"]
        if "width" in config:
            self.width = config["width"]
        if "height" in config:
            self.height = config["height"]
        if "scale" in config:
            self.scale = config["scale"]
        if "smell_cell_size" in config:
            self.smell_cell_size = config["smell_cell_size"]

        if self.smell_cell_size is None:
            self.smell_cell_size = max((self.width * self.height) // 20000, 1)

        if "boundary_wall" in config:
            self.boundary_wall = config["boundary_wall"]
        if "boundary_wall_color" in config:
            self.boundary_wall_color = Color(config["boundary_wall_color"])
        if "boundary_wall_width" in config:
            self.boundary_wall_width = config["boundary_wall_width"]
        if "ground_color" in config:
            self.ground_color = Color(config["ground_color"])
        if "ground_image_filename" in config:
            self.set_ground_image(config["ground_image_filename"], show=False)

        # Now, we create the grid:
        self._grid = Grid(self.width, self.height, self.smell_cell_size)
        self._grid.update_walls(self._walls)

        # Add walls:
        self._add_boundary_walls()
        for wall in config.get("walls", []):
            # Walls are either "boxes" with 4 lines, or a single line:
            if "wtype" not in wall or wall["wtype"] == "box":
                self.add_wall(
                    wall["color"],
                    wall["p1"]["x"],
                    wall["p1"]["y"],
                    wall["p2"]["x"],
                    wall["p2"]["y"],
                    box=True,
                )
            elif wall["wtype"] == "line":
                self.add_wall(
                    wall["color"],
                    wall["p1"]["x"],
                    wall["p1"]["y"],
                    wall["p2"]["x"],
                    wall["p2"]["y"],
                    box=False,
                )

        for bulb in config.get("bulbs", []):
            # bulbs are {x, y, z, color, brightness}
            self._add_bulb(**bulb)

        for food in config.get("food", []):
            # food x, y, standard_deviation
            self._add_food(**food)

        ## Create robot, and add to world:
        for i, robotConfig in enumerate(self.config.get("robots", [])):
            # FIXME: raise if lengths don't match
            if i < len(self._robots):  # already a robot; let's reuse it:
                robot = self._robots[i]
                robot._initialize()
                robot.from_json(robotConfig)
            else:
                robot = Robot(**robotConfig)
                self.add_robot(robot)
        # Create the backend if first time:
        if self._backend is None:
            self._backend = make_backend(self.width, self.height, self.scale)
        # Update the backend if it already existed, but differs in config
        self._backend.update_dimensions(self.width, self.height, self.scale)

    def add_food(self, x, y, standard_deviation):
        """
        Add food at x, y with a brightness of standard_deviation
        (in pixels).
        """
        self._add_food(x, y, standard_deviation)
        self._grid.need_update = True
        self.update()  # request draw
        self.save()

    def _add_food(self, x, y, standard_deviation):
        self._food.append((x, y, standard_deviation))

    def _add_boundary_walls(self):
        """
        Add boundary walls around world.
        """
        if self.boundary_wall:
            p1 = Point(0, 0)
            p2 = Point(0, self.height)
            p3 = Point(self.width, self.height)
            p4 = Point(self.width, 0)
            ## Not a box, but surround area with four boundaries:
            self._walls.extend(
                [
                    Wall(self.boundary_wall_color, None, Line(p1, p2), wtype="boundary"),
                    Wall(self.boundary_wall_color, None, Line(p2, p3), wtype="boundary"),
                    Wall(self.boundary_wall_color, None, Line(p3, p4), wtype="boundary"),
                    Wall(self.boundary_wall_color, None, Line(p4, p1), wtype="boundary"),
                ]
            )
            self._complexity = self._compute_complexity()

    def to_json(self):
        """
        Get the world as a JSON dict.
        """
        config = {
            "seed": self.seed,
            "width": int(self.width),
            "height": int(self.height),
            "scale": self.scale,
            "boundary_wall": self.boundary_wall,  # bool
            "boundary_wall_color": str(self.boundary_wall_color),
            "boundary_wall_width": self.boundary_wall_width,
            "ground_color": str(self.ground_color),
            "ground_image_filename": self.ground_image_filename,
            "quiet": self.quiet,
            "smell_cell_size": self.smell_cell_size,
            "walls": [],
            "bulbs": [],
            "robots": [],
            "food": [],
        }
        for wall in self._walls:
            # Not a boundary wall or robot bounding box:
            if wall.wtype == "wall":
                if len(wall.lines) == 4:
                    # Box:
                    w = {
                        "color": str(wall.color),
                        "p1": {"x": wall.lines[0].p1.x, "y": wall.lines[0].p1.y,},
                        "p2": {"x": wall.lines[2].p1.x, "y": wall.lines[2].p1.y,},
                        "wtype": "box",
                    }
                elif len(wall.lines) == 1:
                    # Line:
                    w = {
                        "color": str(wall.color),
                        "p1": {"x": wall.lines[0].p1.x, "y": wall.lines[0].p1.y,},
                        "p2": {"x": wall.lines[0].p2.x, "y": wall.lines[0].p2.y,},
                        "wtype": "line",
                    }
                else:
                    raise Exception("invalid wall length; should be 1 or 4: %s" % len(wall.lines))
                config["walls"].append(w)

        for bulb in self._bulbs:
            config["bulbs"].append(
                {
                    "color": str(bulb.color),
                    "x": bulb.x,
                    "y": bulb.y,
                    "z": bulb.z,
                    "brightness": bulb.brightness,
                }
            )

        for food in self._food:
            config["food"].append(
                {
                    "x": food[0],
                    "y": food[1],
                    "standard_deviation": food[2],
                }
            )

        for robot in self._robots:
            config["robots"].append(robot.to_json())

        return config

    def save(self):
        """
        Save the current state of the world as the config.
        """
        self.config = self.to_json()

    def save_file(self):
        """
        Save the current state of the world as the config, and
        save it back to disc if it was loaded from disk.
        """
        # First, save internally.
        self.config = self.to_json()
        if self.filename is not None and os.path.exists(self.filename):
            with open(self.filename, "w") as fp:
                json_dump(self.config, fp, sort_keys=True, indent=4)
        else:
            if not self.quiet:
                print("Saved in memory. Use world.save_as('filename') to save to disk.")

    def save_as(self, filename):
        """
        Save the world config JSON as a new file.
        """
        if not filename.endswith(".json"):
            filename = filename + ".json"
        # First, save internally.
        self.config = self.to_json()
        with open(filename, "w") as fp:
            json_dump(self.to_json(), fp, sort_keys=True, indent=4)
        self.config["filename"] = filename
        # Now you can use save():

    def set_ground_image(self, filename, show=True):
        """
        Set the background image
        """
        self.ground_image_filename = filename
        self._reset_ground_image()
        if show:
            self.update(show=False)
            self.draw()  # force

    def _reset_ground_image(self):
        """
        Reset the ground image, in case it changed.
        """
        if self.ground_image_filename is not None:
            self._ground_image = load_image(
                self.ground_image_filename,
                round(self.width * self.scale),
                round(self.height * self.scale),
            )
        else:
            self._ground_image = None

        if self._ground_image is not None:
            self._ground_image_pixels = self._ground_image.load()

    def paste_ground_image(self, image, x, y):
        """
        Paste an image onto the ground image. Requires
        a ground image to have already been set.

        Args:
            * image: a Python Image Library image
            * x: (int) the x coordinate of upper lefthand corner
            * y: (int) the y coordinate of upper lefthand corner
        """
        if self._ground_image:
            self._ground_image.paste(image, (x, y))

    def set_ground_color_at(self, x, y, pen):
        """
        Set the pixel(s) of the ground image at position (x,y). Requires
        a ground image to have already been set.

        Args:
            * x: (int) the x coordinate
            * y: (int) the y coordinate
            * pen: (tuple) the (color, radius) to draw with
        """
        if self._ground_image:
            color, radius = pen
            for i in range(-radius, radius + 1, 1):
                for j in range(-radius, radius + 1, 1):
                    try:
                        self._ground_image_pixels[
                            ((x * self.scale) + i, (y * self.scale) + j)
                        ] = color.to_tuple()
                    except Exception:
                        pass

    def get_ground_color_at(self, x, y, radius=1):
        """
        Get the pixel(s) of the ground image at position (x,y). Requires
        a ground image to have already been set.

        Args:
            * x: (int) the x coordinate
            * y: (int) the y coordinate
            * radius: (int) size of area
        """
        if self._ground_image:
            results = []
            for i in range(-radius, radius + 1, 1):
                for j in range(-radius, radius + 1, 1):
                    results.append(
                        self._ground_image_pixels[
                            ((x + i) * self.scale, (y + j) * self.scale)
                        ]
                    )
            return results

    def set_scale(self, scale):
        """
        Change the scale of the rendered world.
        """
        self.scale = scale
        self._backend.update_dimensions(self.width, self.height, self.scale)
        # Save with config
        self.config["scale"] = self.scale
        self.update(show=False)
        self.draw()  # force

    def watch(self, width=None, height=None):
        # Force update:
        self.update(show=True)
        # Force draw:
        self.draw()
        widget = self.get_widget(width, height)
        display(widget)

    def get_widget(self, width=None, height=None):
        if isinstance(width, int):
            width = "%spx" % width
        if isinstance(height, int):
            height = "%spx" % height
        self._step_display = "notebook"
        self.update()
        return self._backend.get_widget(width=width, height=height)

    def record(self):
        from .watchers import Recorder

        recorder = Recorder(self)
        self._watchers.append(recorder)
        self._recording = True
        return recorder

    def _draw_watchers(self):
        if self._backend is not None:
            self._backend.draw_watcher()
        for watcher in self._watchers:
            watcher.draw()

    def _reset_watchers(self):
        if self._backend is not None:
            self._backend.reset_watcher()
        for watcher in self._watchers:
            watcher.reset()

    def _update_watchers(self):
        if self._backend is not None:
            self._backend.update_watcher()
        for watcher in self._watchers:
            watcher.update()

    def clear_watchers(self):
        self._watchers[:] = []

    def add_bulb(self, color, x, y, z, brightness, name=None):
        """
        Add a bulb to the world.
        """
        self._add_bulb(color, x, y, z, brightness, name)
        self.update()  # request draw
        self.save()

    def _add_bulb(self, color, x, y, z, brightness, name=None):
        name = name if name is not None else "bulb-%s" % (len(self._bulbs) + 1)
        bulb = Bulb(color, x, y, z, brightness, name)
        self._bulbs.append(bulb)

    def add_wall(self, color, x1, y1, x2, y2, box=True, wtype="wall"):
        """
        Add a wall line or box of wall lines.
        """
        p1 = Point(x1, y1)
        p3 = Point(x2, y2)
        if box:
            ## Pairs of points make Line:
            p2 = Point(x2, y1)
            p4 = Point(x1, y2)
            wall = Wall(
                Color(color),
                None,
                Line(p1, p2),
                Line(p2, p3),
                Line(p3, p4),
                Line(p4, p1),
                wtype=wtype,
            )
        else:
            wall = Wall(
                Color(color), None, Line(p1, p3), wtype="wall"
            )
        self._walls.append(wall)
        self._complexity = self._compute_complexity()
        self._grid.update_wall(wall)
        self.update()  # request draw

    def del_robot(self, robot):
        """
        Removed a robot from the world.
        """
        if not isinstance(robot, Robot):
            # Then look it up by index/name/type:
            robot = self.robots[robot]
        for wall in list(self._walls):
            if wall.robot is robot:
                self._walls.remove(wall)
        if robot in self._robots:
            robot.world = None
            self._robots.remove(robot)
        self._complexity = self._compute_complexity()
        self.update()  # request draw

    def _find_random_pose(self, robot):
        """
        Add a robot to the world in a random position.
        """
        pa = random.random() * math.pi * 2
        for i in range(100):
            too_close = False
            px = round(robot.radius + random.random() * (self.width - 2 * robot.radius))
            py = round(
                robot.radius + random.random() * (self.height - 2 * robot.radius)
            )
            for other in self._robots:
                if distance(px, py, other.x, other.y) < robot.radius + other.radius:
                    too_close = True
                    break

            for wall in self._walls:
                for line in wall.lines:
                    dist, location = distance_point_to_line((px, py), line.p1, line.p2)
                    if dist < robot.radius:
                        too_close = True
                        break
            if not too_close:
                return px, py, pa

        raise Exception("Couldn't find a place for robot after 100 tries; giving up")

    def add_robot(self, robot):
        """
        Add a new robot to the world. If the robot's position is 0,0 then
        place it randomly in the world.
        """
        if robot not in self._robots:
            if robot.x == 0 and robot.y == 0:
                robot.x, robot.y, robot.a = self._find_random_pose(robot)
            self._robots.append(robot)
            robot.world = self
            # Bounding lines form a wall:
            if len(robot._bounding_lines) == 0:
                print("WARNING: adding a robot with no body")
            wall = Wall(robot.color, robot, *robot._bounding_lines, wtype="robot")
            self._walls.append(wall)
            self._complexity = self._compute_complexity()
            self.update()
            self.save()
        else:
            raise Exception("Can't add the same robot to a world more than once.")

    def _signal_handler(self, *args, **kwargs):
        """
        Handler for Control+C.
        """
        self._stop = True

    @contextmanager
    def _no_interrupt(self):
        """
        Suspends signal handling execution
        """
        self._stop = False
        try:
            signal.signal(signal.SIGINT, self._signal_handler)
        except ValueError:
            # Cannot do this in a thread
            pass

        try:
            yield None
        finally:
            try:
                signal.signal(signal.SIGINT, DEFAULT_HANDLER)
            except ValueError:
                # Cannot do this in a thread
                pass

    def stop(self):
        self._stop = True
        self.status = "stopped"
        if self._thread is not None:
            print("Stopping thread...")
            self._thread.join()
            self._thread = None

    def run(
        self,
        function=None,
        time_step=None,
        show=True,
        real_time=True,
        show_progress=True,
        quiet=False,
        background=False,
    ):
        """
        Run the simulator until one of the control functions returns True
        or Control+C is pressed.

        Args:
            function - (optional) either a single function that takes the
                world, or a list of functions (or None) that each take
                a robot. If any function returns True, then simulation will
                stop.
            time_step - (optional) time unit to advance the world
            show - (optional) update the watchers
            real_time - (optional) run simulation in real time
            show_progress - (optional) show progress bar
            quiet - (optional) if True, do not show the status message when
                completed
            background - (optional) if True, run in the background.
        """
        time_step = time_step if time_step is not None else self.time_step
        if background:
            if self._thread is None:
                kwargs = {
                    "function": function,
                    "time_step": time_step,
                    "show": show,
                    "real_time": real_time,
                    "show_progress": False,
                    "quiet": True,
                    "background": False
                }
                print("Starting world.run() in background. Use world.stop()")
                self._thread = Thread(target=self.run, kwargs=kwargs)
                self._thread.start()
            else:
                print("The world is already running in the background. Use world.stop()")
        else:
            self.steps(
                float("inf"), function, time_step, show, real_time, show_progress, quiet
            )

    def seconds(
        self,
        seconds=5.0,
        function=None,
        time_step=None,
        show=True,
        real_time=True,
        show_progress=True,
        quiet=False,
    ):
        """
        Run the simulator for N seconds, or until one of the control
        functions returns True or Control+C is pressed.

        Args:
            seconds - (optional) how many simulation seconds to run
            function - (optional) either a single function that takes the
                world, or a list of functions (or None) that each take
                a robot. If any function returns True, then simulation will
                stop.
            time_step - (optional) time unit to advance the world
            show - (optional) update the watchers
            real_time - (optional) run simulation in real time
            show_progress - (optional) show progress bar
            quiet - (optional) if True, do not show the status message when
                completed
        """
        time_step = time_step if time_step is not None else self.time_step
        steps = round(seconds / time_step)
        self.steps(steps, function, time_step, show, real_time, show_progress, quiet)

    def steps(
        self,
        steps=1,
        function=None,
        time_step=None,
        show=True,
        real_time=True,
        show_progress=True,
        quiet=False,
    ):
        """
        Run the simulator for N steps, or until one of the control
        functions returns True or Control+C is pressed.

        Args:
            steps - (optional) either a finite number, or infinity
            function - (optional) either a single function that takes the
                world, or a list of functions (or None) that each take
                a robot. If any function returns True, then simulation will
                stop.
            time_step - (optional) time unit to advance the world
            show - (optional) update the watchers
            real_time - (optional) run simulation in real time
            show_progress - (optional) show progress bar
            quiet - (optional) if True, do not show the status message when
                completed
        """
        self.status = "running"
        time_step = time_step if time_step is not None else self.time_step
        if steps == float("inf"):
            step_iter = count()
        else:
            step_iter = range(steps)
        with self._no_interrupt():
            start_real_time = time.monotonic()
            start_time = self.time
            for step in progress_bar(
                step_iter, show_progress and not quiet, self._step_display
            ):
                if self._stop:
                    self.status = "stopped"
                    break
                if function is not None:
                    if isinstance(function, (list, tuple)):
                        if len(function) < len(self._robots):
                            self._print_once("WARNING: you have not provided a controller function for every robot")
                        # Deterministically run robots round-robin:
                        stop = any(
                            [
                                function[i](self._robots[i])
                                for i in range(len(function))
                                if function[i] is not None
                            ]
                        )
                    else:
                        stop = function(self)
                    if stop:
                        break
                self._step(time_step, show=show, real_time=real_time)

        self.status = "stopped"
        stop_real_time = time.monotonic()
        stop_time = self.time
        speed = (stop_time - start_time) / (stop_real_time - start_real_time)
        if steps > 1 and not quiet and not self.quiet:
            print(
                "Simulation stopped at: %s; speed %s x real time"
                % (format_time(self.time), round(speed, 2))
            )
        if show:
            self.draw()  # force to update any displays

    def _print_once(self, message):
        if message not in self._messages:
            print(message)
            self._messages.append(message)

    def _compute_complexity(self):
        # Proxy for how much drawing
        return sum([len(wall.lines) for wall in self._walls])

    def _step(self, time_step=None, show=True, real_time=True):
        """
        Run the simulator for 1 step.

        Args:
            * time_step: (Number, optional) the time unit to advance the simulation
            * show: (bool) if True, update the watchers
            * real_time: (bool) if True, run in real time, even introducing a
                delay if necessary
        """
        if time_step is not None and not isinstance(time_step, Number):
            raise ValueError(
                "Invalid time_step: %r; should be a number or None" % time_step
            )
        if not isinstance(show, bool):
            raise ValueError("Invalid show: %r; should be a bool" % show)
        if not isinstance(real_time, bool):
            raise ValueError("Invalid real_time: %r; should be a bool" % real_time)

        # Throttle needs to take into account the async update time
        # So as not to overwhelm the system. We give 0.1 time
        # per robot. This can be optimized to reduce the load.

        if self._backend.is_async():
            self._throttle_period = self._backend.get_dynamic_throttle(self)

        time_step = time_step if time_step is not None else self.time_step
        start_time = time.monotonic()
        for robot in self._robots:
            robot._step(time_step)
        self.time += time_step
        self.time = round(self.time, self._time_decimal_places)
        self.update(show)
        self._update_watchers()
        if show:
            now = time.monotonic()
            time_passed = now - start_time
            if real_time:  # real_time is ignored if not show
                sleep_time = self.time_step - time_passed
                # Tries to sleep enough to make even with real time/throttle time:
                # If running faster than real time/throttle period, need to wait some
                if sleep_time >= 0:
                    # Sleep even more for slow-motion:
                    time.sleep(sleep_time)
                # else it is already running slower than real time
            elif not self._backend.is_async():
                # Goal is to keep time_passed less than % of throttle period:
                if time_passed > self._throttle_period * self._show_throttle_percentage:
                    self._throttle_period += time_step

    def update(self, show=True):
        """
        Update the world, robots, and devices. Optionally, draw the
        world.
        """
        ## Update robots:
        self._draw_list = self._overlay_list[:]
        for robot in self._robots:
            robot.update(self._draw_list)
        if show:
            self._request_draw()

    def _request_draw(self):
        """
        Draw the world. This function is throttled
        """
        # Throttle:
        now = time.monotonic()
        time_since_last_call = now - self._time_of_last_call

        if time_since_last_call > self._throttle_period:
            self._time_of_last_call = now
            # End of throttle code

            self.draw()  # force

    def _get_robot_bulbs(self):
        bulbs = []
        for robot in self.robots:
            for device in robot._devices:
                if device.type == "bulb":
                    bulbs.append(device)
        return bulbs

    def _get_light_sources(self, all=False):
        if all:
            return [bulb for bulb in (self._bulbs + self._get_robot_bulbs()) if bulb.state == "on"]
        else:
            return [bulb for bulb in self._bulbs if bulb.state == "on"]

    def draw(self):
        """
        Force a redraw of the world.
        """
        if self._backend is None:
            return

        self._grid.update(self._food)

        with self._backend:
            self._backend.clear()
            self._backend.noStroke()
            if self._ground_image is not None:
                self._backend.draw_image(self._ground_image, 0, 0)
            else:
                self._backend.set_fill(self.ground_color)
                self._backend.draw_rect(0, 0, self.width, self.height)

            if len(self._food) > 0:
                smell = self._grid.get_image()
                smell = smell.resize((int(self.width * self.scale), int(self.height * self.scale)))
                self._backend.image.paste(smell, (0, 0), smell)

            ## Draw bulbs in world (not on robots):
            for bulb in self._get_light_sources(all=False):
                bulb.draw(self._backend)

            ## Draw walls:
            for wall in self._walls:
                if wall.wtype == "wall":
                    c = wall.color
                    if len(wall.lines) == 1:
                        self._backend.strokeStyle(c, 5)
                        self._backend.draw_line(
                            wall.lines[0].p1.x,
                            wall.lines[0].p1.y,
                            wall.lines[0].p2.x,
                            wall.lines[0].p2.y,
                        )
                    else:
                        self._backend.set_fill(c)
                        self._backend.noStroke()
                        self._backend.beginShape()
                        for line in wall.lines:
                            self._backend.vertex(line.p1.x, line.p1.y)
                            self._backend.vertex(line.p2.x, line.p2.y)
                        self._backend.endShape()

                    self._backend.lineWidth(1)
                    self._backend.noStroke()

                elif wall.wtype == "boundary":
                    c = wall.color
                    self._backend.strokeStyle(c, 3)
                    self._backend.draw_line(
                        wall.lines[0].p1.x,
                        wall.lines[0].p1.y,
                        wall.lines[0].p2.x,
                        wall.lines[0].p2.y,
                    )
                    self._backend.lineWidth(1)
                    self._backend.noStroke()

            ## Draw robots:
            for robot in self._robots:
                robot.draw(self._backend)

            text = format_time(self.time)
            self._backend.draw_status(text)

            for items in self._draw_list:
                if len(items) == 1:
                    command = items[0]
                    args = tuple()
                    kwargs = {}
                elif len(items) == 2:
                    command = items[0]
                    args = items[1]
                    kwargs = {}
                elif len(items) == 3:
                    command = items[0]
                    args = items[1]
                    kwargs = items[2]

                self._backend.do_command(command, *args, **kwargs)

        self._draw_watchers()
