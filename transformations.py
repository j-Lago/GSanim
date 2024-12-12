from math import sin, cos, pi, atan2, sqrt
import typing
from copy import copy
from colorsys import hls_to_rgb, rgb_to_hls

from typing import Union

Numeric = Union[float, int]
Coords = Union[list[Numeric, ...], tuple[Numeric, ...]]
Point = tuple[Numeric, Numeric]

# type Numeric = float or int
# type Coords = list[Numeric, ...] or tuple[Numeric, ...]
# type Point = tuple[Numeric, Numeric]


def translate(coords: Coords, offset: Point) -> Coords:
    input_type = type(coords)

    if len(coords) == 3:
        return input_type((coords[0] + offset[0], coords[1] + offset[1], coords[2]))
    else:
        return input_type(v + offset[i % 2] for i, v in enumerate(coords))


def scale(coords: Coords, factor: Numeric | Point, center: Point = (0, 0)) -> Coords:
    input_type = type(coords)

    if len(coords) == 3:
        return input_type(((coords[0]-center[0])*factor+center[0], (coords[1]-center[1])*factor+center[1], coords[2]*factor))
        # return input_type((coords[0], coords[1], coords[2]*factor))
    else:
        if isinstance(factor, float | int):
            # print(factor)
            factor = (factor, factor)
        return input_type(factor[i_xy := i % 2] * (v-center[i_xy]) + center[i_xy] for i, v in enumerate(coords))


def reverse(coords: Coords) -> Coords:
    input_type = type(coords)
    original = copy(coords)
    L = len(original)//2
    pairs = (zip(coords[::2], coords[1::2]))  # (x0, y0, x1, y1, ...) -> ((x0, y0), (x1, y1), ...)
    pairs = ( (original[-i*2], original[-i*2-1])  for i in range(L) )
    coords = (v for pt in pairs for v in pt)  # ((x0, y0), (x1, y1), ...) -> (x0, y0, x1, y1, ...)
    return input_type(coords)

def rotate(coords: Coords, angle: Numeric, center: Point = (0, 0)) -> Coords:
    input_type = type(coords)

    c = cos(angle)
    s = sin(angle)
    x0, y0 = center

    if len(coords) == 3:
        x, y, r = coords
        return input_type(( (dx := x-x0) * c - (dy := y-y0) * s + x0,  dx * s + dy * c + y0, r))
    else:
        pairs = (zip(coords[::2], coords[1::2]))    # (x0, y0, x1, y1, ...) -> ((x0, y0), (x1, y1), ...)
        pairs = (((dx := x-x0) * c - (dy := y-y0) * s + x0, dx * s + dy * c + y0) for x, y in pairs)
        coords = (v for pt in pairs for v in pt)    # ((x0, y0), (x1, y1), ...) -> (x0, y0, x1, y1, ...)
        return input_type(coords)

def denorm_coords(canvas, coords) -> tuple:
    input_type = type(coords)
    base = min(canvas.winfo_width(), canvas.winfo_height()) // 2
    center = (canvas.winfo_width() / 2), (canvas.winfo_height() / 2)
    return input_type(v * (base, -base)[i_xy := k % 2] + center[i_xy] for k, v in enumerate(coords))

def norm_coords(canvas, coords) -> tuple:
    base = min(canvas.winfo_width(), canvas.winfo_height()) // 2
    center = (canvas.winfo_width() / 2), (canvas.winfo_height() / 2)
    return tuple((v - center[i_xy := k % 2]) / (base, -base)[i_xy]  for k, v in enumerate(coords))

def coords_circle_to_oval(coords) -> Coords:
    input_type = type(coords)

    # print(self.coords)
    match len(coords):
        case 3:                         # circulo definido pelo centro e raio coords = (x0, y0, r)
            x0, y0, r = coords

        case 4:                         # circulo definido pelo centro e ponto de intersecção coords = (x0, y0, x1, y1)
            x0, y0, x1, y1 = coords
            r = sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

        case _:
            raise ValueError(f'círculo são definidos por 3 ou 4 valores, foi recebido {len(coords)=}')

    return input_type((x0 - r, y0 - r, x0 + r, y0 + r))


def clip(v: Numeric, v_min: Numeric, v_max) -> Numeric:
    return max(min(v, v_max), v_min)


def rgb_to_hex(rgb) -> str:
    """
        hex_to_rgb((0.49, 1.0, 0.0)) -> '#ff8833'
    """
    all_floats = True
    for v in rgb:
        if not isinstance(v, float):
            all_floats = False
            break

    if all_floats:
        rgb = tuple(clip(int(v*255), 0, 255) for v in rgb)
    else:
        rgb = tuple(clip(v, 0, 255) for v in rgb)
    return f'#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}'


def scale_hsl(rgb, hue: float = 1.0, sat: float = 1.0, lum: float = 1.0) -> tuple[float, ...]:
    h, s, l = rgb_to_hls(*rgb)
    return hls_to_rgb(h*hue, s*sat, l*lum)

def scale_rgb(rgb, factor) -> tuple[float, ...]:

    type_in = type(rgb)

    r, g, b = (hex_to_rgb(rgb) if type_in == str else rgb)

    ret = (r*factor, g*factor, b*factor)

    if type_in == str:
        return rgb_to_hex(ret)
    return ret

def set_hsl(rgb,
            hue: float | None = None,
            sat: float | None = None,
            lum: float | None = None) -> tuple[float, ...]:

    h, s, l = rgb_to_hls(*rgb)

    if hue: h *= hue
    if sat: h *= sat
    if lum: h *= lum

    return hls_to_rgb(h, l, s*sat)


def hex_to_rgb(hex: str) -> tuple[float, ...]:
    """
    hex_to_rgb('#ff8833') -> (0.49, 1.0, 0.0)
    """
    return tuple(int(hex[i:i+2], 16)/255 for i in range(1, len(hex)-1, 2))




class CircularDict(dict):
    def __init__(self, *args):
        super().__init__(*args)
        keys = list(self.keys())
        self._current_key = keys[0]
        self.name = None

    @property
    def key(self):
        return self._current_key

    @key.setter
    def key(self, value):
        self.set_current_key(value)

    def set_current_key(self, value):
        if value not in list(self.keys()):
            raise ValueError(f"a 'key' fornecida não faz parte do dicionário. São chaves válidas {list(self.keys())}")
        self._current_key = value


    @property
    def value(self):
        return self[self._current_key]

    def __next__(self):
        keys = list(self.keys())
        self._current_key = keys[(keys.index(self._current_key) + 1) % len(keys)]
        return self._current_key



def lerp(a, b, t):
    return a+(b-a)*t


def rgb_lerp(c0, c1, t):
    return tuple( lerp(a, b, t) for a, b in zip(c0, c1) )


def hex_lerp(c0: str, c1: str, t):
    c0 = hex_to_rgb(c0)
    c1 = hex_to_rgb(c1)
    return rgb_to_hex(rgb_lerp(c0, c1, t))


def rgb_bezier(c0, c1, c2, t):
    c01 = rgb_lerp(c0, c1, t)
    c12 = rgb_lerp(c1, c2, t)
    return rgb_lerp(c01, c12, t)


def hex_bezier(c0: str, c1: str, c2: str,  t):
    c0 = hex_to_rgb(c0)
    c1 = hex_to_rgb(c1)
    c2 = hex_to_rgb(c2)
    return rgb_to_hex(rgb_bezier(c0, c1, c2, t))