# -*- coding: utf-8 -*-
# *************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2020 Calysto Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
#
# *************************************

import json
import os
import threading
import time

from IPython.display import HTML as display_HTML, Image as display_Image, display
from ipywidgets import (
    Box,
    Button,
    FloatSlider,
    FloatText,
    HTML,
    HBox,
    Image,
    Label,
    Layout,
    Output,
    Text,
    Textarea,
    VBox,
)

from .utils import Point, arange, image_to_gif, image_to_png, progress_bar
from .world import World


def make_attr_widget(obj, map, title, attrs, labels):
    box = VBox()
    children = []
    if title is not None:
        children.append(Label(value=title))
    for i, attr in enumerate(attrs):
        if hasattr(obj, attr) and isinstance(getattr(obj, attr), dict):
            widget = Textarea(description=labels[i])
        else:
            widget = Text(description=labels[i])
        map[labels[i]] = widget
        children.append(widget)
    box.children = children
    return box


class Watcher:
    def __init__(self):
        """
        All watchers need a widget.
        """
        self.widget = None

    def draw(self):
        """
        Some watchers need to be told explicitly when to draw.
        """
        raise NotImplementedError("need to implement watcher.draw()")

    def update(self):
        """
        Some watchers need to be told explicitly when to update
        their state.
        """
        raise NotImplementedError("need to implement watcher.update()")

    def reset(self):
        """
        Some watchers hold state and need to be reset.
        """
        raise NotImplementedError("need to implement watcher.reset()")

    def watch(self):
        """
        This method should return the widget associated with
        the watcher.
        """
        return self.widget


class RobotWatcher(Watcher):
    def __init__(self, robot, size=100, show_robot=True):
        super().__init__()
        self.robot = robot
        self.size = size
        self.show_robot = show_robot
        self.map = {}
        self.attrs = ["name", "x", "y", "direction", "stalled", "tvx", "tva", "state"]
        self.labels = [
            "Name:",
            "X:",
            "Y:",
            "Direction:",
            "Stalled:",
            "Trans vel:",
            "Rotate vel:",
            "State:",
        ]
        widget = make_attr_widget(self.robot, self.map, None, self.attrs, self.labels)
        if self.show_robot:
            css = HTML("<style>img.pixelated {image-rendering: pixelated;}</style>")
            display(css)
            image = Image(layout=Layout(
                width="-webkit-fill-available",
                height="auto",
            ))
            image.add_class("pixelated")
            widget.children = [image] + list(widget.children)

        self.widget = widget
        self.update()
        self.draw()

    def draw(self):
        if self.robot.world is None:
            print("This robot is not in a world")
            return

        if self.show_robot:
            picture = self.robot.world.take_picture()
            start_x = round(
                max(self.robot.x * self.robot.world.scale - self.size / 2, 0)
            )
            start_y = round(
                max(self.robot.y * self.robot.world.scale - self.size / 2, 0)
            )
            rectangle = (
                start_x,
                start_y,
                min(
                    start_x + self.size, self.robot.world.width * self.robot.world.scale
                ),
                min(
                    start_y + self.size,
                    self.robot.world.height * self.robot.world.scale,
                ),
            )
            picture = picture.crop(rectangle)
            self.widget.children[0].value = image_to_png(picture)
        for i in range(len(self.attrs)):
            attr = getattr(self.robot, self.attrs[i])
            if isinstance(attr, dict):
                string = json.dumps(attr, sort_keys=True, indent=2)
            else:
                string = str(attr)
            self.map[self.labels[i]].value = string

    def update(self):
        pass

    def reset(self):
        pass


class AttributesWatcher(Watcher):
    def __init__(self, obj, *attrs, title=None, labels=None):
        super().__init__()
        self.obj = obj
        self.map = {}
        self.attrs = attrs
        self.title = title
        self.labels = labels
        if self.labels is None:
            self.labels = ["%s:" % attr for attr in attrs]
        self.widget = make_attr_widget(self.obj, self.map, title, attrs, labels)
        self.update()
        self.draw()

    def draw(self):
        for i in range(len(self.attrs)):
            self.map[self.labels[i]].value = str(getattr(self.obj, self.attrs[i]))

    def update(self):
        pass

    def reset(self):
        pass


class CameraWatcher:
    def __init__(self, camera, width=None, height=None):
        super().__init__()
        self.camera = camera
        width = "-webkit-fill-available" if width is None else width
        height = "auto" if height is None else height

        self.widget = Image(
            layout=Layout(
                width=width,
                height=height,
            )
        )
        self.widget.add_class("pixelated")
        css = HTML("<style>img.pixelated {image-rendering: pixelated;}</style>")
        display(css)
        # Update and draw:
        self.draw()

    def draw(self):
        picture = self.camera.take_picture()
        self.widget.value = image_to_png(picture)

    def update(self):
        pass

    def reset(self):
        pass


class _Player(threading.Thread):
    """
    Background thread for running a player.
    """

    def __init__(self, controller, time_wait=0.5):
        self.controller = controller
        threading.Thread.__init__(self)
        self.time_wait = time_wait
        self.can_run = threading.Event()
        self.can_run.clear()  # paused
        self.daemon = True  # allows program to exit without waiting for join

    def run(self):
        while True:
            self.can_run.wait()
            self.controller.goto("next")
            time.sleep(self.time_wait)

    def pause(self):
        self.can_run.clear()

    def resume(self):
        self.can_run.set()


class Recorder(Watcher):
    def __init__(self, world, play_rate=0.1):
        super().__init__()
        self.states = []
        self.orig_world = world
        # Copy of the world for creating playback:
        self.world = World(**world.to_json())
        # Copy items needed for playback
        for i in range(len(self.world._robots)):
            # Copy list references:
            self.world._robots[i].text_trace = self.orig_world._robots[i].text_trace
            self.world._robots[i].pen_trace = self.orig_world._robots[i].pen_trace
        self.widget = Player("Time:", self.goto, 0, play_rate)

    def draw(self):
        self.widget.update_length(len(self.states))

    def update(self):
        # Record the states from the real world:
        states = []
        for robot in self.orig_world._robots:
            states.append(
                (
                    robot.x,
                    robot.y,
                    robot.direction,
                    robot.vx,
                    robot.vy,
                    robot.va,
                    robot.stalled,
                )
            )
        self.states.append(states)

    def reset(self):
        self.states = []

    def watch(self, play_rate=0.0):
        self.widget.player.time_wait = play_rate
        return self.widget

    def get_trace(self, robot_index, current_index, max_length):
        # return as [Point(x,y), direction]
        start_index = max(current_index - max_length, 0)
        return [
            (Point(x, y), a)
            for (x, y, a, vx, vy, va, stalled) in [
                state[robot_index]
                for state in self.states[start_index : current_index + 1]
            ]
        ]

    def goto(self, time):
        index = round(time / 0.1)
        # place robots where they go in copy:
        if len(self.states) == 0:
            for i, orig_robot in enumerate(self.orig_world._robots):
                x, y, a, vx, vy, va, stalled = (
                    orig_robot.x,
                    orig_robot.y,
                    orig_robot.direction,
                    orig_robot.vx,
                    orig_robot.vy,
                    orig_robot.va,
                    orig_robot.stalled,
                )
                self.world.robots[i]._set_pose(x, y, a, clear_trace=False)
                self.world.robots[i].vx = vx
                self.world.robots[i].vy = vy
                self.world.robots[i].va = va
                self.world.robots[i].stalled = stalled
                self.world.robots[i].trace[:] = []
        else:
            index = max(min(len(self.states) - 1, index), 0)
            for i, state in enumerate(self.states[index]):
                x, y, a, vx, vy, va, stalled = state
                self.world.robots[i]._set_pose(x, y, a, clear_trace=False)
                self.world.robots[i].vx = vx
                self.world.robots[i].vy = vy
                self.world.robots[i].va = va
                self.world.robots[i].stalled = stalled
                if self.world.robots[i].do_trace:
                    self.world.robots[i].trace = self.get_trace(
                        i, index, self.world.robots[i].max_trace_length
                    )
        self.world.time = time
        if self.world.time == 0:
            # In case it changed:
            self.world.reset_ground_image()
        self.world.update()
        picture = self.world.take_picture()
        return picture

    def save_as(
        self,
        movie_name="aitk_movie",
        start=0,
        stop=None,
        step=0.1,
        loop=0,
        duration=100,
        embed=False,
        mp4=True,
    ):
        """
        Save as animated gif and optionally mp4; show with controls.
        loop - 0 means continually
        duration - in MS
        """
        if stop is None:
            stop = len(self.states) * 0.1

        stop = min(stop, len(self.states) * 0.1)

        frames = []
        for time_step in progress_bar(arange(start, stop, step)):
            # Special function to load as gif, leave fp open
            picture = image_to_gif(self.goto(time_step))
            frames.append(picture)

        if frames:
            # First, save animated gif:
            frames[0].save(
                movie_name + ".gif",
                save_all=True,
                append_images=frames[1:],
                loop=loop,
                duration=duration,
            )
            if not mp4:
                return display_Image(url=movie_name + ".gif", embed=embed)
            else:
                if os.path.exists(movie_name + ".mp4"):
                    os.remove(movie_name + ".mp4")
                retval = os.system(
                    """ffmpeg -i {0}.gif -movflags faststart -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" {0}.mp4""".format(
                        movie_name
                    )
                )
                if retval == 0:
                    return display_HTML(
                        """<video src='{0}.mp4' controls style="width: 100%"></video>""".format(
                            movie_name
                        )
                    )
                else:
                    print(
                        "error running ffmpeg; see console log message or use mp4=False"
                    )


class Player(VBox):
    def __init__(self, title, function, length, play_rate=0.1):
        """
        function - takes a slider value and returns displayables
        """
        self.player = _Player(self, play_rate)
        self.player.start()
        self.title = title
        self.function = function
        self.length = length
        self.output = Output()
        self.position_text = FloatText(value=0.0, layout=Layout(width="100%"))
        self.total_text = Label(
            value="of %s" % round(self.length * 0.1, 1), layout=Layout(width="100px")
        )
        controls = self.make_controls()
        super().__init__([controls, self.output])

    def update_length(self, length):
        self.length = length
        self.total_text.value = "of %s" % round(self.length * 0.1, 1)
        self.control_slider.max = round(max(self.length * 0.1, 0), 1)

    def goto(self, position):
        #### Position it:
        if position == "begin":
            self.control_slider.value = 0.0
        elif position == "end":
            self.control_slider.value = round(self.length * 0.1, 1)
        elif position == "prev":
            if self.control_slider.value - 0.1 < 0:
                self.control_slider.value = round(self.length * 0.1, 1)  # wrap around
            else:
                self.control_slider.value = round(
                    max(self.control_slider.value - 0.1, 0), 1
                )
        elif position == "next":
            if round(self.control_slider.value + 0.1, 1) > round(self.length * 0.1, 1):
                self.control_slider.value = 0  # wrap around
            else:
                self.control_slider.value = round(
                    min(self.control_slider.value + 0.1, self.length * 0.1), 1
                )
        self.position_text.value = round(self.control_slider.value, 1)

    def toggle_play(self, button):
        ## toggle
        if self.button_play.description == "Play":
            self.button_play.description = "Stop"
            self.button_play.icon = "pause"
            self.player.resume()
        else:
            self.button_play.description = "Play"
            self.button_play.icon = "play"
            self.player.pause()

    def make_controls(self):
        button_begin = Button(icon="fast-backward", layout=Layout(width="100%"))
        button_prev = Button(icon="backward", layout=Layout(width="100%"))
        button_next = Button(icon="forward", layout=Layout(width="100%"))
        button_end = Button(icon="fast-forward", layout=Layout(width="100%"))
        self.button_play = Button(
            icon="play", description="Play", layout=Layout(width="100%")
        )
        self.control_buttons = HBox(
            [
                button_begin,
                button_prev,
                self.position_text,
                button_next,
                button_end,
                self.button_play,
            ],
            layout=Layout(width="100%", height="50px"),
        )
        self.control_slider = FloatSlider(
            description=self.title,
            continuous_update=False,
            min=0.0,
            step=0.1,
            max=max(round(self.length * 0.1, 1), 0.0),
            value=0.0,
            readout_format=".1f",
            style={"description_width": "initial"},
            layout=Layout(width="100%"),
        )
        ## Hook them up:
        button_begin.on_click(lambda button: self.goto("begin"))
        button_end.on_click(lambda button: self.goto("end"))
        button_next.on_click(lambda button: self.goto("next"))
        button_prev.on_click(lambda button: self.goto("prev"))
        self.button_play.on_click(self.toggle_play)
        self.control_slider.observe(self.update_slider_control, names="value")
        controls = VBox(
            [
                HBox(
                    [self.control_slider, self.total_text], layout=Layout(height="40px")
                ),
                self.control_buttons,
            ],
            layout=Layout(width="100%"),
        )
        controls.on_displayed(lambda widget: self.initialize())
        return controls

    def initialize(self):
        """
        Setup the displayer ids to map results to the areas.
        """
        results = self.function(self.control_slider.value)
        if not isinstance(results, (list, tuple)):
            results = [results]
        self.displayers = [display(x, display_id=True) for x in results]

    def update_slider_control(self, change):
        """
        If the slider changes the value, call the function
        and update display areas.
        """
        if change["name"] == "value":
            self.position_text.value = self.control_slider.value
            self.output.clear_output(wait=True)
            results = self.function(self.control_slider.value)
            if not isinstance(results, (list, tuple)):
                results = [results]
            for i in range(len(self.displayers)):
                self.displayers[i].update(results[i])
