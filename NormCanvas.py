from __future__ import annotations
import tkinter as tk
from transformations import Coords, Point, denorm_coords
from math import sqrt


class NormCanvas(tk.Canvas):
    def __init__(self, window: tk.Tk, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.window = window


class BoolVar(tk.IntVar):
    def __init__(self, value=False):
        super().__init__()
        self.set(value)

    def __bool__(self):
        return not self.get() == 0