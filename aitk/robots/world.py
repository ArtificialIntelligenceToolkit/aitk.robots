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
import os
import random
import re
import signal
import time
from collections.abc import Sequence
from contextlib import contextmanager
from itertools import count
from numbers import Number

from .backends import make_backend
from .colors import BLACK_50, WHITE
from .robot import Robot
from .utils import (
    Color,
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

    def __init__(self, color, robot, *lines):
        self.color = color
        self.robot = robot
        self.lines = lines

    def __repr__(self):
        return "Wall(%r, %r, %r)" % (self.color, self.robot, self.lines)


class Bulb:
    """
    Class representing lights in the world.
    """

    def __init__(self, color, x, y, z, brightness):
        self.color = Color(color)
        self.x = x
        self.y = y
        self.z = z
        self.brightness = brightness

    def __repr__(self):
        return "Bulb(color:%r, x:%r, y:%r, z:%r, brightness:%r)" % (
            self.color,
            self.x,
            self.y,
            self.z,
            self.brightness,
        )


class RobotList(Sequence):
    def __init__(self, world):
        self.world = world

    def __getitem__(self, item):
        if isinstance(item, int):
            return self.world._robots[item]
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
            for robot in self.world._robots:
                # update name_map
                robot_name = robot.name.lower()
                robot_index = None
                if "-" in robot_name:
                    robot_prefix, robot_index = robot_name.rsplit("-", 1)
                    if robot_index.isdigit():
                        robot_name = robot_prefix
                        robot_index = int(robot_index)
                    else:
                        robot_index = 1
                if robot_name not in name_map:
                    name_map[robot_name] = 1
                else:
                    name_map[robot_name] += 1
                if robot_index is None:
                    robot_index = name_map[robot_name]
                if search_name == robot_name and search_index == robot_index:
                    return robot

        return None

    def __len__(self):
        return len(self.world._robots)

    def __repr__(self):
        return repr(self.world._robots)


class World:
    """
    The Jyrobot simulator world.
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
        **kwargs
    ):
        """
        The Jyrobot simulator world.

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
        }
        if filename is not None:
            config["filename"] = filename
        if ground_image_filename is not None:
            config["ground_image_filename"] = ground_image_filename
        config["walls"] = kwargs.pop("walls", [])
        config["bulbs"] = kwargs.pop("bulbs", [])
        config["robots"] = kwargs.pop("robots", [])
        if len(kwargs) != 0:
            raise AttributeError(
                "unknown arguments for World: %s" % list(kwargs.keys())
            )
        self.show_throttle_percentage = 0.40
        self.time_decimal_places = 1
        self.throttle_period = 0.1
        self.time_of_last_call = 0
        self.step_display = "tqdm"
        self.debug = False
        self.watchers = []
        self._robots = []
        self.backend = None
        self.recording = False
        self.config = config.copy()
        self.initialize()  # default values
        self.reset()  # from config
        self.robots = RobotList(self)

    def __repr__(self):
        return "<World width=%r, height=%r>" % (self.width, self.height)

    def display(self, index=None, size=100):
        """
        Take a picture of the world, or of a robot.
        """
        # FIXME: Make sure it is up to date
        try:
            picture = self.backend.take_picture(self.time)
        except RuntimeError:
            raise Exception("Backend is not ready yet; try again")

        if picture is None:
            return

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

    def info(self):
        """
        Get info about this world, and all of its robots.
        """
        if self.filename:
            print("This world was loaded from %r" % self.filename)
        if len(self._robots) == 0:
            print("This world has no robots.")
        else:
            print("Size: %s x %s" % (self.width, self.height))
            print("Robots:")
            print("-" * 25)
            for i, robot in enumerate(self._robots):
                print("  .robots[%s or %r]: %r" % (i, robot.name, robot))
                robot.info()

    def get_robot(self, item):
        """
        Get the robot by name or index. Equivalent to
        world.robots[item]

        Args:
            * item: (int or string) index or name of robot
        """
        return self.robots[item]

    def switch_backend(self, backend):
        """
        Switch graphic backends. Valid choices are:
            * "jupyter"
            * "svg"
            * "debug"
        """
        self.backend = make_backend(self.width, self.height, self.scale)
        self.backend.update_dimensions(self.width, self.height, self.scale)

    def initialize(self):
        """
        Sets the default values.
        """
        self.draw_list = []
        self.filename = None
        self.quiet = False
        self.seed = 0
        self.width = 500
        self.height = 250
        self.scale = 3.0
        self.stop = False  # should stop?
        self.time_step = 0.10  # seconds
        self.time = 0.0  # seconds
        self.boundary_wall = True
        self.boundary_wall_width = 1
        self.boundary_wall_color = Color(128, 0, 128)
        self.ground_color = Color(0, 128, 0)
        self.ground_image_filename = None
        self.ground_image = None
        self.ground_image_pixels = None
        self.walls = []
        self.bulbs = []
        self.complexity = 0

    def reset(self):
        """
        Reloads the config from initialization, or from
        last save.
        """
        self.initialize()
        self.reset_watchers()
        self.from_json(self.config)
        self.time = 0.0
        for robot in self._robots:
            robot.reset()
            # Re-add the robot's boundaries:
            wall = Wall(robot.color, robot, *robot.bounding_lines)
            self.walls.append(wall)
        self.stop = False  # should stop?
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

        self.add_boundary_walls()

        for wall in config.get("walls", []):
            # Walls are "boxes"... 4 lines:
            self.add_wall(
                wall["color"],
                wall["p1"]["x"],
                wall["p1"]["y"],
                wall["p2"]["x"],
                wall["p2"]["y"],
            )

        for bulb in config.get("bulbs", []):
            # bulbs are {x, y, z, color, brightness}
            self.add_bulb(Bulb(**bulb))

        ## Create robot, and add to world:
        for i, robotConfig in enumerate(self.config.get("robots", [])):
            # FIXME: raise if lengths don't match
            if i < len(self._robots):  # already a robot; let's reuse it:
                robot = self._robots[i]
                robot.initialize()
                robot.from_json(robotConfig)
            else:
                robot = Robot(**robotConfig)
                self.add_robot(robot)
        # Create the backend if first time:
        if self.backend is None:
            self.backend = make_backend(self.width, self.height, self.scale)
        # Update the backend if it already existed, but differs in config
        self.backend.update_dimensions(self.width, self.height, self.scale)

    def clear_boundary_walls(self):
        """
        Remove any boundary walls.
        """
        self.walls[:] = [wall for wall in self.walls if len(wall.lines) > 1]
        self.complexity = self.compute_complexity()

    def add_boundary_walls(self):
        """
        Add boundary walls around world.
        """
        if self.boundary_wall:
            p1 = Point(0, 0)
            p2 = Point(0, self.height)
            p3 = Point(self.width, self.height)
            p4 = Point(self.width, 0)
            ## Not a box, but surround area with four boundaries:
            self.walls.extend(
                [
                    Wall(self.boundary_wall_color, None, Line(p1, p2)),
                    Wall(self.boundary_wall_color, None, Line(p2, p3)),
                    Wall(self.boundary_wall_color, None, Line(p3, p4)),
                    Wall(self.boundary_wall_color, None, Line(p4, p1)),
                ]
            )
            self.complexity = self.compute_complexity()

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
            "walls": [],
            "bulbs": [],
            "robots": [],
        }
        for wall in self.walls:
            if len(wall.lines) == 4 and wall.robot is None:
                w = {
                    "color": str(wall.color),
                    "p1": {"x": wall.lines[0].p1.x, "y": wall.lines[0].p1.y,},
                    "p2": {"x": wall.lines[2].p1.x, "y": wall.lines[2].p1.y,},
                }
                config["walls"].append(w)

        for bulb in self.bulbs:
            config["bulbs"].append(
                {
                    "color": str(bulb.color),
                    "x": bulb.x,
                    "y": bulb.y,
                    "z": bulb.z,
                    "brightness": bulb.brightness,
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
        self.reset_ground_image()
        if show:
            self.update(show=False)
            self.draw()  # force

    def reset_ground_image(self):
        """
        Reset the ground image, in case it changed.
        """
        if self.ground_image_filename is not None:
            self.ground_image = load_image(
                self.ground_image_filename,
                round(self.width * self.scale),
                round(self.height * self.scale),
            )
        else:
            self.ground_image = None

        if self.ground_image is not None:
            self.ground_image_pixels = self.ground_image.load()

    def paste_ground_image(self, image, x, y):
        """
        Paste an image onto the ground image. Requires
        a ground image to have already been set.

        Args:
            * image: a Python Image Library image
            * x: (int) the x coordinate of upper lefthand corner
            * y: (int) the y coordinate of upper lefthand corner
        """
        if self.ground_image:
            self.ground_image.paste(image, (x, y))

    def set_ground_color_at(self, x, y, pen):
        """
        Set the pixel(s) of the ground image at position (x,y). Requires
        a ground image to have already been set.

        Args:
            * x: (int) the x coordinate
            * y: (int) the y coordinate
            * pen: (tuple) the (color, radius) to draw with
        """
        if self.ground_image:
            color, radius = pen
            for i in range(-radius, radius + 1, 1):
                for j in range(-radius, radius + 1, 1):
                    self.ground_image_pixels[
                        ((x * self.scale) + i, (y * self.scale) + j)
                    ] = color.to_tuple()

    def get_ground_color_at(self, x, y, radius=1):
        """
        Get the pixel(s) of the ground image at position (x,y). Requires
        a ground image to have already been set.

        Args:
            * x: (int) the x coordinate
            * y: (int) the y coordinate
            * radius: (int) size of area
        """
        if self.ground_image:
            results = []
            for i in range(-radius, radius + 1, 1):
                for j in range(-radius, radius + 1, 1):
                    results.append(
                        self.ground_image_pixels[
                            ((x + i) * self.scale, (y + j) * self.scale)
                        ]
                    )
            return results

    def set_scale(self, scale):
        """
        Change the scale of the rendered world.
        """
        self.scale = scale
        self.backend.update_dimensions(self.width, self.height, self.scale)
        # Save with config
        self.config["scale"] = self.scale
        self.update(show=False)
        self.draw()  # force

    def watch(self, width=None, height=None):
        self.step_display = "notebook"
        self.update()
        return self.backend.watch(width, height)

    def record(self):
        from .watchers import Recorder

        recorder = Recorder(self)
        self.watchers.append(recorder)
        self.recording = True
        return recorder

    def plot(
        self, function, x_label="x", y_label="y", title=None,
    ):
        from .plots import Plot

        if title is None:
            title = "Jyrobot World"

        plot = Plot(self, function, x_label, y_label, title)
        self.watchers.append(plot)
        return plot

    def draw_watchers(self):
        if self.backend is not None:
            self.backend.draw_watcher()
        for watcher in self.watchers:
            watcher.draw()

    def reset_watchers(self):
        if self.backend is not None:
            self.backend.reset_watcher()
        for watcher in self.watchers:
            watcher.reset()

    def update_watchers(self):
        if self.backend is not None:
            self.backend.update_watcher()
        for watcher in self.watchers:
            watcher.update()

    def del_watchers(self):
        self.watchers[:] = []

    def add_bulb(self, color, x, y, z, brightness):
        """
        Add a bulb to the world.
        """
        bulb = Bulb(color, x, y, z, brightness)
        self.bulbs.append(bulb)
        self.update()  # request draw

    def add_wall(self, color, x1, y1, x2, y2):
        """
        Add a box of walls.
        """
        p1 = Point(x1, y1)
        p2 = Point(x2, y1)
        p3 = Point(x2, y2)
        p4 = Point(x1, y2)
        ## Pairs of points make Line:
        wall = Wall(
            Color(color), None, Line(p1, p2), Line(p2, p3), Line(p3, p4), Line(p4, p1)
        )
        self.walls.append(wall)
        self.complexity = self.compute_complexity()
        self.update()  # request draw

    def del_robot(self, robot):
        """
        Removed a robot from the world.
        """
        if not isinstance(robot, Robot):
            # Then look it up by index/name/type:
            robot = self.robots[robot]
        for wall in list(self.walls):
            if wall.robot is robot:
                self.walls.remove(wall)
        if robot in self._robots:
            robot.world = None
            self._robots.remove(robot)
        self.complexity = self.compute_complexity()
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

            for wall in self.walls:
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
                robot.x, robot.y, robot.direction = self._find_random_pose(robot)
            self._robots.append(robot)
            robot.world = self
            # Bounding lines form a wall:
            if len(robot.bounding_lines) == 0:
                print("WARNING: adding a robot with no body")
            wall = Wall(robot.color, robot, *robot.bounding_lines)
            self.walls.append(wall)
            self.complexity = self.compute_complexity()
            self.update()
            self.save()
        else:
            raise Exception("Can't add the same robot to a world more than once.")

    def _signal_handler(self, *args, **kwargs):
        """
        Handler for Control+C.
        """
        self.stop = True

    @contextmanager
    def _no_interrupt(self):
        """
        Suspends signal handling execution
        """
        self.stop = False
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

    def run(
        self,
        function=None,
        time_step=None,
        show=True,
        real_time=True,
        show_progress=True,
        quiet=False,
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
        """
        time_step = time_step if time_step is not None else self.time_step
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
        time_step = time_step if time_step is not None else self.time_step
        if steps == float("inf"):
            step_iter = count()
        else:
            step_iter = range(steps)
        with self._no_interrupt():
            start_real_time = time.monotonic()
            start_time = self.time
            for step in progress_bar(
                step_iter, show_progress and not quiet, self.step_display
            ):
                if self.stop:
                    break
                if function is not None:
                    if isinstance(function, (list, tuple)):
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
                self.step(time_step, show=show, real_time=real_time)

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

    def compute_complexity(self):
        # Proxy for how much drawing
        return sum([len(wall.lines) for wall in self.walls])

    def step(self, time_step=None, show=True, real_time=True):
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

        if self.backend.is_async():
            self.throttle_period = self.backend.get_dynamic_throttle(self)

        time_step = time_step if time_step is not None else self.time_step
        start_time = time.monotonic()
        for robot in self._robots:
            robot.step(time_step)
        self.time += time_step
        self.time = round(self.time, self.time_decimal_places)
        self.update(show)
        self.update_watchers()
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
            elif not self.backend.is_async():
                # Goal is to keep time_passed less than % of throttle period:
                if time_passed > self.throttle_period * self.show_throttle_percentage:
                    self.throttle_period += time_step

    def update(self, show=True):
        """
        Update the world, robots, and devices. Optionally, draw the
        world.
        """
        ## Update robots:
        self.draw_list = []
        for robot in self._robots:
            robot.update(self.draw_list)
        if show:
            self.request_draw()

    def request_draw(self):
        """
        Draw the world. This function is throttled
        """
        # Throttle:
        now = time.monotonic()
        time_since_last_call = now - self.time_of_last_call

        if time_since_last_call > self.throttle_period:
            self.time_of_last_call = now
            # End of throttle code

            self.draw()  # force

    def draw(self):
        """
        Force a redraw of the world.
        """
        if self.backend is None:
            return

        with self.backend:
            self.backend.clear()
            self.backend.noStroke()
            if self.ground_image is not None:
                self.backend.draw_image(self.ground_image, 0, 0)
            else:
                self.backend.set_fill(self.ground_color)
                self.backend.draw_rect(0, 0, self.width, self.height)
            ## Draw walls:
            for wall in self.walls:
                if len(wall.lines) >= 1 and wall.robot is None:
                    c = wall.color
                    self.backend.noStroke()
                    self.backend.set_fill(c)
                    self.backend.beginShape()
                    for line in wall.lines:
                        self.backend.vertex(line.p1.x, line.p1.y)
                        self.backend.vertex(line.p2.x, line.p2.y)

                    self.backend.endShape()

            ## Draw bulbs:
            for bulb in self.bulbs:
                c = bulb.color
                self.backend.noStroke()
                self.backend.set_fill(c)
                self.backend.draw_circle(bulb.x, bulb.y, bulb.brightness * 5)

            ## Draw borders:
            for wall in self.walls:
                c = wall.color
                if len(wall.lines) == 1:
                    self.backend.strokeStyle(c, 3)
                    self.backend.draw_line(
                        wall.lines[0].p1.x,
                        wall.lines[0].p1.y,
                        wall.lines[0].p2.x,
                        wall.lines[0].p2.y,
                    )
                    self.backend.lineWidth(1)
                    self.backend.noStroke()

            ## Draw robots:
            for robot in self._robots:
                robot.draw(self.backend)

            text = format_time(self.time)
            pos_x, pos_y = (
                self.backend.char_height,
                self.height - self.backend.char_height * 2,
            )

            self.backend.set_fill(BLACK_50)
            self.backend.draw_rect(
                pos_x,
                pos_y,
                self.backend.char_width * len(text),
                self.backend.char_height + 1,
            )
            self.backend.set_fill(WHITE)
            self.backend.text(text, pos_x, pos_y)

            for command, args in self.draw_list:
                self.backend.do_command(command, *args)

        self.draw_watchers()
