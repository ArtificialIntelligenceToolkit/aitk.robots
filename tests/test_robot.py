# -*- coding: utf-8 -*-
# *************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2020 Calysto Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
#
# *************************************

import aitk.robots
from aitk.robots import Color, Robot, Scribbler, World


def test_robot():
    robot = Robot()

    assert (robot.x, robot.y, robot.direction) == (0, 0, 0)


def test_robot_in_world():
    robot = Scribbler()

    assert (robot.x, robot.y, robot.direction) == (0, 0, 0)

    world = World()
    world.add_robot(robot)
    assert (robot.x, robot.y, robot.direction) != (0, 0, 0)

    x, y, direction = (10, 10, 0)
    robot.set_pose(x, y, direction)
    assert (robot.x, robot.y, robot.direction) != (0, 0, 0)

    robot.move(1, 0)
    assert robot.tvx == 2.0
    assert robot.vx == 0.0
    world.steps(1, real_time=False, show=False)
    assert x + 0.2 == robot.x  # can travel .2 with this velocity ramp
    world.steps(1, real_time=False, show=False)
    assert x + 0.6 == robot.x  # can travel .4 with this velocity ramp
    world.steps(1, real_time=False, show=False)
    assert x + 1.2 == robot.x  # can travel .8 with this velocity ramp
    world.steps(1, real_time=False, show=False)
    assert x + 2.0 == robot.x  # can travel max 1/step with this velocity ramp
    world.steps(1, real_time=False, show=False)
    assert x + 3.0 == robot.x  # can travel max 1/step with this velocity ramp

    x, y, direction = (10, 10, 0)
    robot.set_pose(x, y, direction)
    robot.tvx = 1
    robot.vx = 1
    world.steps(1, real_time=False, show=False)
    assert x + 1 == robot.x

    x, y, direction = (10, 10, 0)
    robot.set_pose(x, y, direction)
    robot.tvx = 1
    robot.vx = 1
    world.steps(100, real_time=False, show=False)
    assert x + 100 == robot.x


def test_robot_pen():
    world = aitk.robots.load_world("soccer")
    robot = world.robots[0]

    x, y, direction = (10, 10, 0)
    robot.set_pose(x, y, direction)

    pixels = world.get_ground_color_at(x, y, 1)
    assert len(pixels) == 3 ** 2
    assert pixels != [Color("blue").to_tuple() for i in range(9)]

    robot.pen_down("blue", 3)
    world.step(real_time=False)

    pixels = world.get_ground_color_at(x, y, 1)
    assert len(pixels) == 3 ** 2
    assert pixels == [Color("blue").to_tuple() for i in range(9)]
