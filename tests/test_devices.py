# -*- coding: utf-8 -*-
# *************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2020 Calysto Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
#
# *************************************

from aitk.robots import (
    Robot,
    Scribbler,
    World,
    Camera,
    GroundCamera,
    RangeSensor,
)

from PIL import Image

def test_camera():
    world = World()
    robot = Scribbler(x=100, y=100, a=90)
    device = Camera()

    robot.add_device(device)
    world.add_robot(robot)

    device.display()
    device.get_image()
    device.get_widget()
    device.watch()

def test_rangefinder():
    world = World()
    robot = Scribbler(x=100, y=100, a=90)
    device = RangeSensor()

    robot.add_device(device)
    world.add_robot(robot)

    device.get_widget()
    device.watch()

def test_ground_camera():
    world = World(width=780, height=496,
        ground_image_filename="soccer-780x496.png",
    )
    robot = Scribbler(x=100, y=100, a=90)
    device = GroundCamera()

    robot.add_device(device)
    world.add_robot(robot)

    device.display()
    device.get_image()
    device.get_widget()
    device.watch()
