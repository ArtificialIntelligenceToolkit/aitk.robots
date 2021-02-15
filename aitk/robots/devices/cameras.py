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

from ..utils import Color


class Camera:
    def __init__(
        self,
        width=256,
        height=128,
        angle=60,
        colorsFadeWithDistance=0.5,
        sizeFadeWithDistance=1.0,
        reflectGround=True,
        reflectSky=False,
        max_range=1000,
        name="camera",
        samples=1,
        **kwargs
    ):
        """
        A camera device.

        Args:
            * width: (int) width of camera in pixels
            * height: (int) height of camera in pixels
            * angle: (number) width of camera field of view in degrees. Can be
                180 or even 360 for wide angle cameras.
            * colorsFadeWithDistance: (bool) colors get darker with distance?
            * sizeFadeWithDistance: (bool) size get smaller with distance?
            * reflectGround: (bool) ground reflects for 3D point cloud
            * reflectSky: (bool) sky reflects for 3D point cloud
            * max_range: (int) maximum range of camera
            * name: (str) the name of the camera
            * samples: (int) how many pixels should it sample

        Note: currently the camera faces forward. TODO.
        """
        config = {
            "width": width,
            "height": height,
            "angle": angle,
            "colorsFadeWithDistance": colorsFadeWithDistance,
            "sizeFadeWithDistance": sizeFadeWithDistance,
            "reflectGround": reflectGround,
            "reflectSky": reflectSky,
            "max_range": max_range,
            "name": name,
            "samples": samples,
        }
        self.robot = None
        self.initialize()
        self.from_json(config)

    def initialize(self):
        # FIXME: camera is fixed at (0,0) facing forward
        self.type = "camera"
        self.time = 0.0
        self.cameraShape = [256, 128]
        self.max_range = 1000
        self.samples = 1
        self.name = "camera"
        # 0 = no fade, 1.0 = max fade
        self.colorsFadeWithDistance = 0.5
        self.sizeFadeWithDistance = 1.0
        self.reflectGround = True
        self.reflectSky = False
        self.set_fov(60)  # degrees
        self.reset()

    def reset(self):
        self.hits = [[] for i in range(self.cameraShape[0])]

    def from_json(self, config):
        if "width" in config:
            self.cameraShape[0] = config["width"]
        if "height" in config:
            self.cameraShape[1] = config["height"]

        if "colorsFadeWithDistance" in config:
            self.colorsFadeWithDistance = config["colorsFadeWithDistance"]
        if "sizeFadeWithDistance" in config:
            self.sizeFadeWithDistance = config["sizeFadeWithDistance"]
        if "reflectGround" in config:
            self.reflectGround = config["reflectGround"]
        if "reflectSky" in config:
            self.reflectSky = config["reflectSky"]
        if "angle" in config:
            self.set_fov(config["angle"])  # degrees
        if "max_range" in config:
            self.max_range = config["max_range"]
        if "samples" in config:
            self.samples = config["samples"]
        if "name" in config:
            self.name = config["name"]

    def to_json(self):
        return {
            "class": self.__class__.__name__,
            "width": self.cameraShape[0],
            "height": self.cameraShape[1],
            "colorsFadeWithDistance": self.colorsFadeWithDistance,
            "sizeFadeWithDistance": self.sizeFadeWithDistance,
            "reflectGround": self.reflectGround,
            "reflectSky": self.reflectSky,
            "angle": self.angle * 180 / math.pi,  # save in degrees
            "max_range": self.max_range,
            "samples": self.samples,
            "name": self.name,
        }

    def __repr__(self):
        return "<Camera %r size=(%r,%r), angle=%r>" % (
            self.name,
            self.cameraShape[0],
            self.cameraShape[1],
            round(self.angle * 180 / math.pi, 2),
        )

    def watch(self):
        from ..watchers import CameraWatcher

        if self.robot is None or self.robot.world is None:
            print("ERROR: can't watch until added to robot, and robot is in world")
            return None

        watcher = CameraWatcher(self)
        self.robot.world.watchers.append(watcher)
        # Return the widget:
        return watcher.widget

    def step(self, time_step):
        pass

    def _get_visible_area(self):
        """
        What are the ranges of the field of view?
        Return a list of (p1, p2) that represent lines
        across the background, from front to back,
        down the edges of the field of view.
        """
        step = 1
        all_points = []
        for angle in [self.angle / 2, -self.angle / 2]:
            points = []
            dx, dy = self.robot.rotate_around(0, 0, step, self.robot.direction + angle)
            cx, cy = self.robot.x, self.robot.y
            x, y = cx, cy
            for i in range(0, self.max_range, step):
                points.append((x, y))
                x += dx
                y += dy
            all_points.append(points)
        return zip(*all_points)

    def update(self, draw_list=None):
        """
        Cameras operate in a lazy way: they don't actually update
        until needed because they are so expensive.
        """
        if self.robot.world.debug and draw_list is not None:
            draw_list.append(("set_stroke_style", (Color("white"),)))
            p = self.robot.rotate_around(
                self.robot.x,
                self.robot.y,
                self.max_range,
                self.robot.direction + self.angle / 2,
            )
            draw_list.append(("draw_line", (self.robot.x, self.robot.y, p[0], p[1])))
            p = self.robot.rotate_around(
                self.robot.x,
                self.robot.y,
                self.max_range,
                self.robot.direction - self.angle / 2,
            )
            draw_list.append(("draw_line", (self.robot.x, self.robot.y, p[0], p[1])))

    def _update(self):
        # Update timestamp:
        self.time = self.robot.world.time
        for i in range(self.cameraShape[0]):
            angle = i / self.cameraShape[0] * self.angle - self.angle / 2
            self.hits[i] = self.robot.cast_ray(
                self.robot.x,
                self.robot.y,
                math.pi / 2 - self.robot.direction - angle,
                1000,
            )

    def draw(self, backend):
        """
        Currently, cameras are fixed at 0,0 and face forwards.
        """
        backend.set_fill(Color(0, 64, 0))
        backend.strokeStyle(None, 0)
        backend.draw_rect(5.0, -3.33, 1.33, 6.33)
        p = self.robot.rotate_around(0, 0, self.max_range, -self.angle / 2,)
        backend.draw_line(0, 0, p[0], p[1])
        p = self.robot.rotate_around(0, 0, self.max_range, self.angle / 2,)
        backend.draw_line(0, 0, p[0], p[1])

    def find_closest_wall(self, hits):
        for hit in reversed(hits):  # reverse make it closest first
            if hit.height < 1.0:  # skip non-walls
                continue
            return hit.distance
        return float("inf")

    def get_ground_color(self, area, i, j):
        if self.robot.world.ground_image is not None and area is not None:
            # i is width ray (camera width),
            # j is distance (height of camera/2, 64 to 128)
            dist = round(
                ((self.cameraShape[1] - j) / self.cameraShape[1] / 3) * len(area)
            )
            visible_width_points = area[dist]
            p1, p2 = visible_width_points
            # get a position i/width on line
            # minx, maxx = sorted([p1[0], p2[0]])
            # miny, maxy = sorted([p1[1], p2[1]])
            spanx = abs(p1[0] - p2[0])
            spany = abs(p1[1] - p2[1])
            if p1[0] < p2[0]:
                x = round(
                    (p1[0] + spanx * (1.0 - i / self.cameraShape[0]))
                    * self.robot.world.scale
                )
            else:
                x = round(
                    (p1[0] - spanx * (1.0 - i / self.cameraShape[0]))
                    * self.robot.world.scale
                )
            if p1[1] < p2[1]:
                y = round(
                    (p1[1] + spany * (1.0 - i / self.cameraShape[0]))
                    * self.robot.world.scale
                )
            else:
                y = round(
                    (p1[1] - spany * (1.0 - i / self.cameraShape[0]))
                    * self.robot.world.scale
                )
            # find that pixel
            if (0 <= x < (self.robot.world.width - 1) * self.robot.world.scale) and (
                0 <= y < (self.robot.world.height - 1) * self.robot.world.scale
            ):
                # c = Color(*self.robot.world.ground_image_pixels[(x, y)])
                # self.robot.world.ground_image_pixels[(x, y)] = (0, 0, 0)
                # return c

                sum = Color(0)
                count = 0
                # Need more sampling as distance increases:
                for j in range(self.samples):
                    try:
                        c = Color(*self.robot.world.ground_image_pixels[(x + j, y)])
                        # self.robot.world.ground_image_pixels[(x + j, y)] = (0, 0, 0)
                        count += 1
                    except Exception:
                        continue
                    sum += c
                return sum / count

        return self.robot.world.ground_color

    def take_picture(self, type="color"):
        try:
            from PIL import Image
        except ImportError:
            print("Pillow (PIL) module not available; take_picture() unavailable")
            return

        # Lazy; only get the data when we need it:
        self._update()
        if self.robot.world.ground_image is not None:
            area = list(self._get_visible_area())
        else:
            area = None
        pic = Image.new("RGBA", (self.cameraShape[0], self.cameraShape[1]))
        pic_pixels = pic.load()
        # FIXME: probably should have a specific size rather than scale it to world
        size = max(self.robot.world.width, self.robot.world.height)
        hcolor = None
        # draw non-robot walls first:
        for i in range(self.cameraShape[0]):
            hits = [hit for hit in self.hits[i] if hit.height == 1.0]  # only walls
            if len(hits) == 0:
                continue
            hit = hits[-1]  # get closest
            high = None
            hcolor = None
            if hit:
                # FIXME: need to figure out what height would actually be at this distance
                distance_ratio = max(min(1.0 - hit.distance / size, 1.0), 0.0)
                s = max(
                    min(1.0 - hit.distance / size * self.sizeFadeWithDistance, 1.0), 0.0
                )
                sc = max(
                    min(1.0 - hit.distance / size * self.colorsFadeWithDistance, 1.0),
                    0.0,
                )
                if type == "color":
                    r = hit.color.red * sc
                    g = hit.color.green * sc
                    b = hit.color.blue * sc
                elif type == "depth":
                    r = 255 * distance_ratio
                    g = 255 * distance_ratio
                    b = 255 * distance_ratio
                else:
                    avg = (hit.color.red + hit.color.green + hit.color.blue) / 3.0
                    r = avg * sc
                    g = avg * sc
                    b = avg * sc
                hcolor = Color(r, g, b)
                high = (1.0 - s) * self.cameraShape[1]
            else:
                high = 0

            horizon = self.cameraShape[1] / 2
            for j in range(self.cameraShape[1]):
                dist = max(min(abs(j - horizon) / horizon, 1.0), 0.0)
                if j < high / 2:  # sky
                    if type == "depth":
                        if self.reflectSky:
                            color = Color(255 * dist)
                        else:
                            color = Color(0)
                    elif type == "color":
                        color = Color(0, 0, 128)
                    else:
                        color = Color(128 / 3)
                    pic_pixels[i, j] = color.to_tuple()
                elif j < self.cameraShape[1] - high / 2:  # hit
                    if hcolor is not None:
                        pic_pixels[i, j] = hcolor.to_tuple()
                else:  # ground
                    if type == "depth":
                        if self.reflectGround:
                            color = Color(255 * dist)
                        else:
                            color = Color(0)
                    elif type == "color":
                        color = self.get_ground_color(area, i, j)
                    else:
                        color = Color(128 / 3)
                    pic_pixels[i, j] = color.to_tuple()

        # Other robots, draw on top of walls:
        self.obstacles = {}
        for i in range(self.cameraShape[0]):
            closest_wall_dist = self.find_closest_wall(self.hits[i])
            hits = [hit for hit in self.hits[i] if hit.height < 1.0]  # obstacles
            for hit in hits:
                if hit.distance > closest_wall_dist:
                    # Behind this wall
                    break
                distance_ratio = max(min(1.0 - hit.distance / size, 1.0), 0.0)
                s = max(
                    min(1.0 - hit.distance / size * self.sizeFadeWithDistance, 1.0), 0.0
                )
                sc = max(
                    min(1.0 - hit.distance / size * self.colorsFadeWithDistance, 1.0),
                    0.0,
                )
                distance_to = self.cameraShape[1] / 2 * (1.0 - sc)
                # scribbler was 30, so 0.23 height ratio
                # height is ratio, 0 to 1
                height = round(hit.height * self.cameraShape[1] / 2.0 * s)
                if type == "color":
                    r = hit.color.red * sc
                    g = hit.color.green * sc
                    b = hit.color.blue * sc
                elif type == "depth":
                    r = 255 * distance_ratio
                    g = 255 * distance_ratio
                    b = 255 * distance_ratio
                else:
                    avg = (hit.color.red + hit.color.green + hit.color.blue) / 3.0
                    r = avg * sc
                    g = avg * sc
                    b = avg * sc
                hcolor = Color(r, g, b)
                horizon = self.cameraShape[1] / 2
                self.record_obstacle(
                    hit.robot,
                    i,
                    self.cameraShape[1] - 1 - round(distance_to),
                    self.cameraShape[1] - height - 1 - 1 - round(distance_to),
                )
                if not hit.robot.has_image():
                    for j in range(height):
                        pic_pixels[
                            i, self.cameraShape[1] - j - 1 - round(distance_to)
                        ] = hcolor.to_tuple()
        self.show_obstacles(pic)
        return pic

    def show_obstacles(self, image):
        # FIXME: show back to front
        # FIXME: how to show when partially behind wall?
        for data in self.obstacles.values():
            if data["robot"].has_image():
                # the angle to me + offset for graphics + the robot angle:
                radians = (
                    math.atan2(
                        data["robot"].x - self.robot.x, data["robot"].y - self.robot.y
                    )
                    + math.pi / 2
                    + data["robot"].direction
                )
                degrees = round(radians * 180 / math.pi)
                picture = data["robot"].get_image(degrees)  # degrees
                x1, y1 = data["min_x"], data["min_y"]  # noqa: F841
                x2, y2 = data["max_x"], data["max_y"]
                try:  # like too small
                    picture.thumbnail((x2 - x1, 10000))  # to keep aspect ratio
                    # picture = picture.resize((x2 - x1, y2 - y1))
                    x3 = x2 - picture.height
                    y3 = y2 - picture.width
                    image.paste(picture, (x3, y3), picture)
                except Exception:
                    print("Exception in processing image")

    def record_obstacle(self, robot, x, y1, y2):
        if robot.name not in self.obstacles:
            self.obstacles[robot.name] = {
                "robot": robot,
                "max_x": float("-inf"),
                "max_y": float("-inf"),
                "min_x": float("inf"),
                "min_y": float("inf"),
            }
        self.obstacles[robot.name]["max_x"] = max(
            self.obstacles[robot.name]["max_x"], x
        )
        self.obstacles[robot.name]["min_x"] = min(
            self.obstacles[robot.name]["min_x"], x
        )
        self.obstacles[robot.name]["max_y"] = max(
            self.obstacles[robot.name]["max_y"], y1, y2
        )
        self.obstacles[robot.name]["min_y"] = min(
            self.obstacles[robot.name]["min_y"], y1, y2
        )

    def get_point_cloud(self):
        depth_pic = self.take_picture("depth")
        depth_pixels = depth_pic.load()
        color_pic = self.take_picture("color")
        color_pixels = color_pic.load()
        points = []
        for x in range(self.cameraShape[0]):
            for y in range(self.cameraShape[1]):
                dist_color = depth_pixels[x, y]
                color = color_pixels[x, y]
                if dist_color[0] != 255:
                    points.append(
                        [
                            self.cameraShape[0] - x - 1,
                            self.cameraShape[1] - y - 1,
                            dist_color[0],
                            color[0],
                            color[1],
                            color[2],
                        ]
                    )
        return points

    def set_fov(self, angle):
        """
        Set the field of view angle in degrees of the camera.

        Args:
            * angle: (number) angle in degrees of field of view
        """
        # given in degrees
        # save in radians
        # scale = min(max(angle / 6.0, 0.0), 1.0)
        self.angle = angle * math.pi / 180.0
        # self.sizeFadeWithDistance = scale
        self.reset()

    def set_size(self, width, height):
        """
        Set the height and width of the camera in pixels.

        Args:
            * width: (int) width of camera in pixels
            * height: (int) height of camera in pixels
        """
        self.cameraShape[0] = width
        self.cameraShape[1] = height
        self.reset()

    def set_angle(self, angle):
        """
        Set the field of view angle of the camera.

        Args:
            * angle: (number) angle in degrees of field of view
        """
        self.set_fov(angle)

    def set_max(self, max_range):
        """
        Set the maximum distance the camera can see.

        Args:
            * max_range: (number) distance (in CM) the camera can see
        """
        self.max_range = max_range

    def set_width(self, width):
        """
        Set the width of the camera in pixels.

        Args:
            * width: (int) width of camera in pixels
        """
        self.cameraShape[0] = width
        self.reset()

    def set_height(self, height):
        """
        Set the height of the camera in pixels.

        Args:
            * height: (int) height of camera in pixels
        """
        self.cameraShape[1] = height
        self.reset()

    def set_name(self, name):
        """
        Set the name of the camera.

        Args:
            * name: (str) the name of the camera
        """
        self.name = name

    def get_name(self):
        """
        Get the name of the camera.
        """
        return self.name

    def get_width(self):
        """
        Get the width in pixels of the camera.
        """
        return self.cameraShape[0]

    def get_height(self):
        """
        Get the height in pixels of the camera.
        """
        return self.cameraShape[1]

    def get_angle(self):
        """
        Get the field of view angle in degrees.
        """
        return self.angle * 180 / math.pi

    def get_max(self):
        """
        Get the maximum distance in CM the camera can see.
        """
        return self.max_range


class GroundCamera:
    def __init__(self, width=15, height=15, name="ground-camera", **kwargs):
        """
        A downward-facing camera device.

        Args:
            * width: (int) width of camera in pixels
            * height: (int) height of camera in pixels
            * name: (str) the name of the camera
        """
        config = {
            "width": width,
            "height": height,
            "name": name,
        }
        self.robot = None
        self.initialize()
        self.from_json(config)

    def initialize(self):
        self.type = "ground-camera"
        self.time = 0.0
        self.cameraShape = [15, 15]
        self.name = "ground-camera"

    def from_json(self, config):
        if "width" in config:
            self.cameraShape[0] = config["width"]
        if "height" in config:
            self.cameraShape[1] = config["height"]
        if "name" in config:
            self.name = config["name"]

    def to_json(self):
        return {
            "class": self.__class__.__name__,
            "width": self.cameraShape[0],
            "height": self.cameraShape[1],
            "name": self.name,
        }

    def __repr__(self):
        return "<GroundCamera %r size=(%r,%r)>" % (
            self.name,
            self.cameraShape[0],
            self.cameraShape[1],
        )

    def watch(self):
        from ..watchers import CameraWatcher

        if self.robot is None or self.robot.world is None:
            print("ERROR: can't watch until added to robot, and robot is in world")
            return None

        watcher = CameraWatcher(self)
        self.robot.world.watchers.append(watcher)
        # Return the widget:
        return watcher.widget

    def step(self, time_step):
        """
        Cameras operate in a lazy way: they don't actually update
        until needed because they are so expensive.
        """
        pass

    def update(self, draw_list=None):
        """
        Cameras operate in a lazy way: they don't actually update
        until needed because they are so expensive.
        """
        pass

    def draw(self, backend):
        left = -(self.cameraShape[0] // 2) / self.robot.world.scale
        upper = -(self.cameraShape[1] // 2) / self.robot.world.scale
        right = (self.cameraShape[0] // 2) / self.robot.world.scale
        lower = (self.cameraShape[1] // 2) / self.robot.world.scale
        backend.strokeStyle(Color(128), 1)
        backend.draw_line(left, upper, right, upper)
        backend.draw_line(right, upper, right, lower)
        backend.draw_line(right, lower, left, lower)
        backend.draw_line(left, lower, left, upper)

    def take_picture(self):
        # FIXME: would be faster to trim image down
        # before rotating
        center = (
            self.robot.x * self.robot.world.scale,
            self.robot.y * self.robot.world.scale,
        )
        rotated_image = self.robot.world.ground_image.rotate(
            (self.robot.direction - math.pi / 4 * 6) * (180 / math.pi), center=center,
        )
        left = center[0] - self.cameraShape[0] // 2
        right = center[0] + self.cameraShape[0] // 2
        upper = center[1] - self.cameraShape[1] // 2
        lower = center[1] + self.cameraShape[1] // 2
        return rotated_image.crop((left, upper, right, lower))
