import tkinter
from abc import ABC, abstractmethod
from NormCanvas import NormCanvas
from time import time
from primitive import Primitive


class Animation(ABC):
    def __init__(self, canvas: NormCanvas, frame_delay: int = 10):
        self._t_init = time()
        self._t_start = 0.0
        self._frame_count = 0

        self.canvas = canvas
        self.frame_delay = frame_delay

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
        self.refresh(t, dt, frame_count)

        self.canvas.window.after(self.frame_delay, self.loop)

    # @abstractmethod
    # def binds(self):
    #     pass
    #     window.bind('<Right>', lambda event: call_function_1())
    #     window.bind('<Left>',  lambda event: call_function_2())

    @abstractmethod
    def refresh(self, t: float, dt: float, frame_count: int):
        pass







