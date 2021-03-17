# coding: utf-8
# *************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2020 Calysto Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
#
# *************************************

import os

import aitk.robots


def test_AvoidObstacles():
    world = aitk.robots.World(width=200, height=200, scale=5.0)
    world.add_wall("blue", 80, 50, 100, 150)
    robot = aitk.robots.Scribbler(x=50, y=100, a=0)
    robot.add_device(
        aitk.robots.RangeSensor(
            position=(6, -6), width=57.3, max=20, a=90, name="left-ir"
        )
    )
    robot.add_device(
        aitk.robots.RangeSensor(
            position=(6, 6), width=57.3, max=20, a=90, name="right-ir"
        )
    )
    world.add_robot(robot)

    robot.state["timer"] = 0
    robot.state["debug"] = False

    def avoid(robot):
        left = robot[0].get_distance()
        right = robot[1].get_distance()
        if (
            left == robot[0].get_max()
            and right == robot[1].get_max()
            and robot.state["timer"] == 0
        ):
            # front clear, move forward
            robot.move(0.5, 0)
            if robot.state["debug"]:
                print("F", end="")
        elif robot.state["timer"] > 0 and robot.state["timer"] < 5:
            # timer triggered, continue current rotation
            robot.state["timer"] += 1
            if robot.state["debug"]:
                print("T", end="")
        elif left < robot[0].get_max():
            # obstacle on left, turn right, trigger timer
            robot.move(0, -0.4)
            robot.state["timer"] = 1
            if robot.state["debug"]:
                print("R", end="")
        elif right < robot[1].get_max():
            # obstacle on right, turn left, trigger timer
            robot.move(0, 0.4)
            robot.state["timer"] = 1
            if robot.state["debug"]:
                print("L", end="")
        else:
            # reset timer to zero
            robot.state["timer"] = 0
            if robot.state["debug"]:
                print("t", end="")

    world.reset()
    world.seconds(20, [avoid], show=False, show_progress=False, quiet=True)

    assert (robot.x, robot.y, robot.a) == (
        66.81695517040795,
        86.31424688270995,
        9.189646003293854,
    )


def test_Coverage():
    world = aitk.robots.World(width=200, height=200, scale=5.0, seed=12345)
    robot = aitk.robots.Scribbler(x=100, y=100, a=90, max_trace_length=60)
    robot.add_device(
        aitk.robots.RangeSensor(
            position=(6, -6), max=20, a=0, width=57.3, name="left-ir"
        )
    )
    robot.add_device(
        aitk.robots.RangeSensor(
            position=(6, 6), max=20, a=0, width=57.3, name="right-ir"
        )
    )
    world.add_robot(robot)
    world.update()
    world.save()

    from math import floor

    class Grid(object):
        """This class creates a grid of locations on top of a simulated world
        to monitor how much of the world has been visited. Each grid location
        is initally set to 0 to indicate that it is unvisited, and is updated
        to 1, once it has been visited."""

        def __init__(self, grid_width, world_width):
            self.grid_width = grid_width
            self.world_width = world_width
            self.grid = []
            for i in range(self.grid_width):
                self.grid.append([0] * self.grid_width)

        def show(self):
            """Print a representation of the grid."""
            for i in range(self.grid_width - 1, -1, -1):
                for j in range(self.grid_width):
                    print("%3d" % self.grid[i][j], end=" ")
                print()
            print()

        def update(self, x, y):
            """In the simulator, the origin is at the top-left corner.
            Update the appropriate grid location."""
            size = self.world_width / self.grid_width
            col = floor(x / size)
            row = int(self.grid_width) - floor(y / size) - 1
            self.grid[row][col] += 1

        def analyze_visits(self):
            """Calculate the percentage of visited cells in the grid."""
            cells_visited = 0
            for i in range(self.grid_width):
                for j in range(self.grid_width):
                    if self.grid[i][j] > 0:
                        cells_visited += 1
            percent_visited = cells_visited / self.grid_width ** 2
            return percent_visited

    from random import random

    def wander(robot):
        left = robot[0].get_distance()
        right = robot[1].get_distance()
        x, y, a = robot.get_pose()
        robot.state["grid"].update(x, y)
        # Save the robot's location only once per second
        if (
            left == robot[0].get_max()
            and right == robot[1].get_max()
            and robot.state["timer"] == 0
        ):
            # front clear, move random direction
            rotate = 1
            if random() < 0.5:
                rotate = -1
            robot.move(0.5, rotate * random())
            if robot.state["debug"]:
                robot.speak("F")
        elif robot.state["timer"] > 0 and robot.state["timer"] < 5:
            # timer triggered, continue current rotation
            robot.state["timer"] += 1
            if robot.state["debug"]:
                robot.speak("timer %d" % (robot.state["timer"]))
        elif left < robot[0].get_max():
            # obstacle on left, turn right, trigger timer
            robot.move(0, -0.4)
            robot.state["timer"] = 1
            if robot.state["debug"]:
                robot.speak("R")
        elif right < robot[1].get_max():
            # obstacle on right, turn left, trigger timer
            robot.move(0, 0.4)
            robot.state["timer"] = 1
            if robot.state["debug"]:
                robot.speak("L")
        else:
            # reset timer to zero
            robot.state["timer"] = 0
            if robot.state["debug"]:
                robot.speak("reset")

    robot.state["timer"] = 0
    robot.state["grid"] = Grid(10, 200)
    robot.state["debug"] = True

    world.reset()

    world.seconds(
        20, [wander], real_time=False, show=False, show_progress=False, quiet=False
    )

    assert (robot.x, robot.y, robot.a) == (
        30.656653293840662,
        72.00201171758178,
        7.551282672853182,
    )

    g = robot.state["grid"]
    g.show()
    g.analyze_visits()


def test_SeekLight():
    HERE = os.path.abspath(os.path.dirname(__file__))
    WORLD = os.path.join(HERE, "worlds", "LightEnclosed")

    world = aitk.robots.load_world(WORLD)
    robot = world.robots[0]

    def seekLight(robot):
        left_ir = robot["left-ir"].get_distance()
        right_ir = robot["right-ir"].get_distance()
        left_light = robot["left-light"].get_brightness()
        right_light = robot["right-light"].get_brightness()
        total_light = left_light + right_light
        diff_light = left_light - right_light
        if total_light > 2.5:
            print("\nFound light!")
            return True
        elif (
            left_ir == robot[0].get_max()
            and right_ir == robot[1].get_max()
            and robot.state["timer"] == 0
        ):
            if abs(diff_light) <= robot.state["light-diff"]:
                # move forward
                robot.move(0.5, 0)
                # if robot.state["debug"]: print("F",end="")
            elif diff_light < 0:
                # light stronger to right
                robot.move(0.2, -0.1)
                # if robot.state["debug"]: print("r",end="")
            else:
                # light stronger to left
                robot.move(0.2, 0.1)
                # if robot.state["debug"]: print("l", end="")
        elif robot.state["timer"] > 0 and robot.state["timer"] < 5:
            # timer triggered, continue current rotation
            robot.state["timer"] += 1
            # if robot.state["debug"]: print("T",end="")
        elif left_ir < robot[0].get_max():
            # obstacle on left, turn right, trigger timer
            robot.move(0.1, -0.4)
            robot.state["timer"] = 1
            # if robot.state["debug"]: print("R",end="")
        elif right_ir < robot[1].get_max():
            # obstacle on right, turn left, trigger timer
            robot.move(0.1, 0.4)
            robot.state["timer"] = 1
            # if robot.state["debug"]: print("L",end="")
        else:
            # reset timer to zero
            robot.state["timer"] = 0
            # if robot.state["debug"]: print("t",end="")

    world.robots[0].set_random_pose()
    world.update()

    robot.state["debug"] = True
    robot.state["timer"] = 0
    robot.state["light-diff"] = 0.02

    robot.set_max_trace_length(120)

    world.seconds(120, [seekLight], real_time=False)

    assert (robot.x, robot.y, robot.a) == (
        94.3013170828803,
        57.43618704593252,
        7.67892160132764,
    )
