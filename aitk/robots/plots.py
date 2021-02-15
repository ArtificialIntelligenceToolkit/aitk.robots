# -*- coding: utf-8 -*-
# *************************************
# aitk.robots: Python robot simulator
#
# Copyright (c) 2020 Calysto Developers
#
# https://github.com/ArtificialIntelligenceToolkit/aitk.robots
#
# *************************************

from bqplot import Axis, Figure, LinearScale, Lines


class Plot:
    def __init__(self, obj, function, x_label="x", y_label="y", title=None):
        """
        Function takes an object, and returns (x,y) point.
        """
        self.obj = obj
        self.x_label = x_label
        self.y_label = y_label
        self.function = function
        self.x_values = []
        self.y_values = []
        self.title = title

        x_sc = LinearScale()
        y_sc = LinearScale()

        line = Lines(
            x=[], y=[], scales={"x": x_sc, "y": y_sc}, stroke_width=3, colors=["red"]
        )

        ax_x = Axis(scale=x_sc, label=x_label)
        ax_y = Axis(scale=y_sc, orientation="vertical", label=y_label)

        self.widget = Figure(marks=[line], axes=[ax_x, ax_y], title=self.title)

    def watch(self):
        return self.widget

    def draw(self):
        with self.widget.marks[0].hold_sync():
            self.widget.marks[0].x = self.x_values
            self.widget.marks[0].y = self.y_values

    def update(self):
        x, y = self.function(self.obj)
        self.x_values.append(x)
        self.y_values.append(y)

    def reset(self):
        self.x_values = []
        self.y_values = []
