from NormCanvas import NormCanvas
from math import sin, cos, pi, atan2, sqrt
from primitive import Primitive


class GraphicVec2(Primitive):
    def __init__(self,
                 real: float, imag: float,
                 canvas: NormCanvas,
                 origin: tuple | complex = complex(0.0, 0.0),
                 arrow_fade_threshold=0.1,
                 arrowshape=(25, 28, 8),
                 stroke='#000000',
                 width=8,
                 **line_kwargs):


        vec = complex(real, imag)
        origin = origin if isinstance(origin, complex) else complex(*origin)

        # if line_kwargs.pop('transforms', None):
        #     raise ValueError("'GraphicVec2' não possui o parâmetro 'transforms'")

        coords = (origin.real, origin.imag, origin.real+vec.real, origin.imag+vec.imag)
        super().__init__(canvas, shape='line', coords=coords, width=width, arrowshape=arrowshape, arrow='last', stroke=stroke, **line_kwargs, dont_draw=True)

        # self.vec = vec
        # self.origin = origin
        self.arrow_fade_threshold = arrow_fade_threshold
        self._arrowshape_init = arrowshape

    def to_complex(self):
        real = self.coords[2] - self.coords[0]
        imag = self.coords[3] - self.coords[1]
        return complex(real, imag)

    def from_complex(self, vec: complex, center=complex(0.0, 0.0)):
        self.coords = (center.real, center.imag, center.real+vec.real, center.imag+vec.imag)

    # @property
    # def _handle(self):
    #     return self.line._handle

    # @property
    # def coords(self):
    #     return tuple((self.origin.real, self.origin.imag, self.vec.real, self.vec.imag))

    # @coords.setter
    # def coords(self, coords):
    #     match len(coords):
    #         case 2:
    #             self.vec = complex(coords[0], coords[1])
    #         case 4:
    #             self.origin = complex(coords[0], coords[1])
    #             self.vec = complex(coords[2]-coords[0], coords[3]-coords[1])
    #         case _: raise ValueError(f"coord' deve ser 2 (x, y) ou 4 (x0, y0, x, y) elementos" )
    #
    #     self._canvas.coords(self._handle, self.denorm_coords())



    def draw(self,**kwargs):

        dx = self.coords[2] - self.coords[0]
        dy = self.coords[3] - self.coords[1]
        amp = abs(sqrt(dx**2 + dy**2))

        # self._canvas.coords(self._handle, self.denorm_coords())
        # self._canvas.itemconfig(self._handle, **kwargs)

        if amp < self.arrow_fade_threshold:
            arrowshape = tuple(int(e * amp / self.arrow_fade_threshold) for e in self._arrowshape_init)
        else:
            arrowshape = self._arrowshape_init

        self._canvas.itemconfig(self._handle, arrowshape=arrowshape)

        super().draw()





    # def set_visibility(self, visible):
    #     if visible:
    #         self.canvas.itemconfig(self.line, state='normal')
    #     else:
    #         self.canvas.itemconfig(self.line, state='hidden')
