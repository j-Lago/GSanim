import tkinter
from abc import ABC, abstractmethod

from typing import Self

from NormCanvas import NormCanvas
from time import time, sleep
from primitive import Primitive
from threading import Event

class AxisLimits:
    def __init__(self, lmin: float = None, lmax: float = None, autoscale: bool = False):
        self.max = lmax
        self.min = lmin

        if lmin is None and lmax is not None:
            self.min = -self.max

        self.autoscale = not autoscale and (lmax is None or lmin is None)

class PlotLimits:
    def __init__(self,
                 xmin: float = None,
                 xmax: float = None,
                 ymin: float = None,
                 ymax: float = None,
                 xautoscale: bool = False,
                 yautoscale: bool = False,
                 twinx: Self = None
                 ):
        self.x = AxisLimits(xmin, xmax, xautoscale)
        self.y = AxisLimits(ymin, ymax, yautoscale)
        self.twinx = twinx




class Animation(ABC):
    def __init__(self, canvas: NormCanvas, frame_delay: int = 10):
        self._t_init = time()
        self._t_start = 0.0
        self._frame_count = 0
        self.freeze = False

        self.canvas = canvas
        self.frame_delay = frame_delay

        self._close_next_refresh = False

        def close_next_refresh():
            self._close_next_refresh = True
        self.canvas.window.protocol("WM_DELETE_WINDOW", close_next_refresh)


    def create_primitive(self, *args, **kwargs) -> Primitive:
        return Primitive(self.canvas,  *args, **kwargs)

    def _start_frame_time_counter(self) -> (float, float, int):
        t = time() - self._t_init
        dt = t - self._t_start

        self._t_start = t
        self._frame_count += 1
        return t, dt, self._frame_count



    def loop(self):
        t, dt, frame_count = self._start_frame_time_counter()
        if not self.freeze:
            self.refresh(t, dt, frame_count)

        if not self._close_next_refresh:
            self.canvas.window.after(self.frame_delay, self.loop)
        else:
            self.canvas.window.destroy()
            del self


    # @abstractmethod
    # def binds(self):
    #     pass
    #     window.bind('<Right>', lambda event: call_function_1())
    #     window.bind('<Left>',  lambda event: call_function_2())

    @abstractmethod
    def refresh(self, t: float, dt: float, frame_count: int):
        pass







