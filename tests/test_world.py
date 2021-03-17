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
from aitk.robots import World, Scribbler


def test_world():
    world = World()

    assert world.width == 500
    assert world.height == 250


def test_soccer_world():
    world = aitk.robots.load_world("soccer")

    assert world.width == 780
    assert world.height == 496
    assert world.ground_image_filename == "soccer-780x496.png"

    robot = world.robots[0]
    picture = robot["camera"].get_image()

    assert picture.size == (64, 32)

def test_recorder():
    world = World()
    robot = Scribbler()
    world.add_robot(robot)

    recorder = world.record()

    def wander(robot):
        if robot.get_time() == 0:
            robot.forward(1)
        if robot.stalled:
            robot.reverse()

    world.seconds(60, [wander], real_time=False)

    recorder.watch()

    widget = recorder.get_widget()

    recorder.goto(0)
