from primitive import Primitive, PrimitivesGroup
from assets import assets, cl, contrast_color_scale
from transformations import translate, rotate, scale, reverse, rgb_to_hex, hex_to_rgb, scale_hsl, scale_rgb,set_hsl, clip, \
    CircularDict
from math import pi
from typing import Literal
from collections.abc import Iterator, Iterable
from NormCanvas import NormCanvas
from gvec import GraphicVec2



def mit_draw(canvas, prims, s_coils_per_phase, r_coils_per_phase):

    create_stator(canvas, prims, s_coils_per_phase)
    create_rotor(canvas, prims, r_coils_per_phase)
    create_fields(canvas, prims)


def create_rotor(canvas, prims, coils_per_phase, phases=3):


    prims['rotor'] = []
    prims['rotor']['core'] = []
    prims['rotor']['core']['outer'] = Primitive(canvas, **assets[name := 'rotor_outer'], name=name)
    prims['rotor']['core']['slots'] = circular_pattern(canvas, ['rotor_cutout', 'rotor_cutout_outline'], pattern=phases * coils_per_phase)
    prims['rotor']['shaft'] = Primitive(canvas, **assets[name := 'shaft'], name=name)
    prims['rotor']['shaft']['keyway'] = [Primitive(canvas, **assets[name := 'keyway'], name=name),
                                         Primitive(canvas, **assets[name := 'keyway_outline'], name=name)]


    prims['rotor']['coil'] = []
    prims['rotor']['coil']['in'] = []
    prims['rotor']['coil']['out'] = []
    prims['rotor']['coil']['in']['x'] = circular_pattern(canvas, ['rotor_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=0, dir_filter='in'))
    prims['rotor']['coil']['out']['x'] = circular_pattern(canvas, ['rotor_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=0, dir_filter='out'))
    prims['rotor']['coil']['in']['y'] = circular_pattern(canvas, ['rotor_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=1, dir_filter='in'))
    prims['rotor']['coil']['out']['y'] = circular_pattern(canvas, ['rotor_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=1, dir_filter='out'))
    prims['rotor']['coil']['in']['z'] = circular_pattern(canvas, ['rotor_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=2, dir_filter='in'))
    prims['rotor']['coil']['out']['z'] = circular_pattern(canvas, ['rotor_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=2, dir_filter='out'))
    # prims['rotor']['coil']['x'] = circular_pattern(canvas, ['rotor_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, 0))
    # prims['rotor']['coil']['y'] = circular_pattern(canvas, ['rotor_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, 1))
    # prims['rotor']['coil']['z'] = circular_pattern(canvas, ['rotor_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, 2))
    for phase in 'xyz':
        for node in prims.filter_matching_keys('coil', phase):
            node.stroke = scale_rgb(cl[phase], contrast_color_scale)
            node.fill = cl[phase]


    prims['rotor']['current'] = []
    prims['rotor']['current']['in'] = []
    prims['rotor']['current']['out'] = []
    prims['rotor']['current']['in']['x'] = circular_pattern(canvas, ['in_r'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=0, dir_filter='in'))
    prims['rotor']['current']['out']['x'] = circular_pattern(canvas, ['out_r'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=0, dir_filter='out'))
    prims['rotor']['current']['in']['y'] = circular_pattern(canvas, ['in_r'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=1, dir_filter='in'))
    prims['rotor']['current']['out']['y'] = circular_pattern(canvas, ['out_r'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=1, dir_filter='out'))
    prims['rotor']['current']['in']['z'] = circular_pattern(canvas, ['in_r'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=2, dir_filter='in'))
    prims['rotor']['current']['out']['z'] = circular_pattern(canvas, ['out_r'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=2, dir_filter='out'))
    for phase in ('xyz'):
        for node in prims.filter_matching_keys('current', phase):
            node.stroke = scale_rgb(cl[phase], contrast_color_scale)
            node.fill = scale_rgb(cl[phase], contrast_color_scale)



def create_fields(canvas: NormCanvas, prims: PrimitivesGroup):
    prims['stator']['field'] = []
    prims['stator']['field']['vec'] = []
    for i, ph in enumerate(('abcs')):
        prims['stator']['field']['vec'][ph] = GraphicVec2(0.4, 0.0, canvas, stroke=cl[ph], transforms=(rotate, 2 * pi / 3 * i), name='field_vec_' + ph)

    prims['rotor']['field'] = []
    prims['rotor']['field']['vec'] = []
    for i, ph in enumerate(('xyzr')):
        prims['rotor']['field']['vec'][ph] = GraphicVec2(0.4, 0.0, canvas, stroke=cl[ph], transforms=(rotate, 2 * pi / 3 * i), name='field_vec_' + ph)

    prims['stator']['field']['lines'] = []
    for i, ph in enumerate('abcs'):
        prims['stator']['field']['lines'][ph] = create_flux_from_quarter(canvas, orientation=2 * pi / 3 * i, color=cl[ph])
        prims['stator']['field']['lines'][ph].visible = False

    prims['rotor']['field']['lines'] = []
    for i, ph in enumerate('xyzr'):
        prims['rotor']['field']['lines'][ph] = create_flux_from_quarter(canvas, orientation=2 * pi / 3 * i, color=cl[ph])
        prims['rotor']['field']['lines'][ph].visible = False


    prims['rotor']['field'].append(Primitive(canvas,'circle', (0.0, 0.0, 0.015), fill=cl['airgap'], stroke=cl['outline'], name='center'))


def create_stator(canvas: NormCanvas, prims: PrimitivesGroup, coils_per_phase: int, phases:int=3):
    prims['stator'] = []

    prims['stator']['core'] = []
    prims['stator']['core']['mount'] = [
        Primitive(canvas, **assets[name := 'mount'], name=name),
        Primitive(canvas, **assets[name := 'mount'], name=name + '.mirrored', transforms=(scale, (-1.0, 1.0))),
    ]
    prims['stator']['core']['outer'] = Primitive(canvas, **assets[name := 'stator_outer'], name=name)
    prims['stator']['core']['inner'] = Primitive(canvas, **assets[name := 'stator_inner'], name=name)

    prims['stator']['core']['slots'] = circular_pattern(canvas, ['stator_cutout', 'stator_cutout_outline'], pattern=phases * coils_per_phase)

    prims['stator']['coil'] = []
    prims['stator']['coil']['in'] = []
    prims['stator']['coil']['out'] = []
    prims['stator']['coil']['in']['a'] = circular_pattern(canvas, ['stator_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=0, dir_filter='in'))
    prims['stator']['coil']['out']['a'] = circular_pattern(canvas, ['stator_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=0, dir_filter='out'))
    prims['stator']['coil']['in']['b'] = circular_pattern(canvas, ['stator_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=1, dir_filter='in'))
    prims['stator']['coil']['out']['b'] = circular_pattern(canvas, ['stator_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=1, dir_filter='out'))
    prims['stator']['coil']['in']['c'] = circular_pattern(canvas, ['stator_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=2, dir_filter='in'))
    prims['stator']['coil']['out']['c'] = circular_pattern(canvas, ['stator_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=2, dir_filter='out'))
    # prims['stator']['coil']['a'] = circular_pattern(canvas, ['stator_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, 0))
    # prims['stator']['coil']['b'] = circular_pattern(canvas, ['stator_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, 1))
    # prims['stator']['coil']['c'] = circular_pattern(canvas, ['stator_esp'], pattern=IndexFilter3ph(phases * coils_per_phase, 2))
    for phase in 'abc':
        for node in prims.filter_matching_keys('coil', phase):
            node.stroke = scale_rgb(cl[phase], contrast_color_scale)
            node.fill = cl[phase]

    prims['stator']['current'] = []
    prims['stator']['current']['in'] = []
    prims['stator']['current']['out'] = []
    prims['stator']['current']['in']['a'] = circular_pattern(canvas, ['in_s'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=0, dir_filter='in'))
    prims['stator']['current']['out']['a'] = circular_pattern(canvas, ['out_s'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=0, dir_filter='out'))
    prims['stator']['current']['in']['b'] = circular_pattern(canvas, ['in_s'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=1, dir_filter='in'))
    prims['stator']['current']['out']['b'] = circular_pattern(canvas, ['out_s'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=1, dir_filter='out'))
    prims['stator']['current']['in']['c'] = circular_pattern(canvas, ['in_s'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=2, dir_filter='in'))
    prims['stator']['current']['out']['c'] = circular_pattern(canvas, ['out_s'], pattern=IndexFilter3ph(phases * coils_per_phase, phase=2, dir_filter='out'))
    for phase in ('abc'):
        for node in prims.filter_matching_keys('current', phase):
            node.stroke = scale_rgb(cl[phase], contrast_color_scale)
            node.fill = scale_rgb(cl[phase], contrast_color_scale)

    # prims['stator']['coil_front'] = create_circular_pattern(canvas, ['stator_esp_front'], pattern=ThreePhaseSkipIndexIterator(phases*coils_per_phase))
    # prims['stator']['coil_front'].stipple = 'gray50'
    # prims['stator']['coil_front'].fill = '#00ffaa'


def create_flux_from_quarter(canvas, orientation=0.0, color: str | None = None):
    orientation += pi
    prims = [
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(reverse, ), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(scale, (-1, 1)), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(scale, (-1, -1)), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(reverse, ), (scale, (1, -1)), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(reverse, ), (scale, {'factor': (0.8, 0.77), 'center': (0.0, -0.50571428571428571428571428571429)}    ), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(scale, (-1, 1)), (scale, {'factor': (0.8, 0.77), 'center': (0.0, -0.50571428571428571428571428571429)}), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(scale, (-1, -1)), (scale, {'factor': (0.8, 0.77), 'center': (0.0, 0.50571428571428571428571428571429)}), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(reverse, ), (scale, (1, -1)), (scale, {'factor': (0.8, 0.77), 'center': (0.0, 0.50571428571428571428571428571429)}), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(reverse, ), (scale, {'factor': (.65, .5), 'center': (0.0, -0.50571428571428571428571428571429)}), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(scale, (-1, 1)), (scale, {'factor': (.65, .5), 'center': (0.0, -0.50571428571428571428571428571429)}), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(scale, (-1, -1)), (scale, {'factor': (.65, .5), 'center': (0.0, 0.50571428571428571428571428571429)}), (rotate, orientation)])),
        (Primitive(canvas, **assets['quarter_flux'], transforms=[(reverse,), (scale, (1, -1)), (scale, {'factor': (.65, .5), 'center': (0.0, 0.50571428571428571428571428571429)}), (rotate, orientation)])),
    ]
    if color:
        for p in prims:
            p.stroke = color

    return prims

def circular_pattern(canvas,
                     assets_keys: str | list,
                     pattern: Iterator | int  # iterator para filtrar elementos de cada fase ou int. caso seja int, iterator será range(int)
                     ):

    if isinstance(assets_keys, str):
        assets_keys = [assets_keys]

    if isinstance(pattern, int):
        pattern = range(pattern)

    n = len(pattern)
    pitch = 2 * pi / n
    start = pi if (n//6) % 2 != 0 else pitch/2+pi
    return [*(Primitive(canvas, **assets[name := key],
                        name=name + f'.rotated({start + pitch * i * 180 / pi:3.0f}°)',
                        transforms=(rotate, start + pitch * i)) for i in pattern for key in assets_keys)]


class IndexFilter3ph:
    def __init__(self, limit, phase=0, dir_filter: Literal['both', 'in', 'out'] = 'both'):

        if limit == 24 and dir_filter != 'both':
            dir_filter = 'out' if dir_filter == 'in' else 'in'

        if dir_filter == 'out':
            self.dir_skip_mult = 2
            self.dir_phase = limit//2
            self.ret_limit = limit//2//3
        elif dir_filter == 'in':
            self.dir_skip_mult = 2
            self.dir_phase = 0
            self.ret_limit = limit//2//3
        else:
            self.dir_skip_mult = 1
            self.dir_phase = 0
            self.ret_limit = limit//3

        self.phase = limit-1 + (limit//3)*phase
        if limit == 6:
            self.phase += 1
        if limit == 24:
            self.phase += 11

        self.phase = (self.phase + self.dir_phase ) % limit
        self.limit = limit



    def __len__(self):
        return self.limit

    def __iter__(self):
        self.returned = 0
        self.i = 0
        return self

    def __next__(self):
        ret = (self.i + self.phase) % self.limit
        if self.returned >= self.ret_limit:
            raise StopIteration
        inc = 1 if self.i % (self.limit//2) < (self.limit//6-1) else (1+self.limit//3)*self.dir_skip_mult

        self.i += inc
        self.returned += 1
        return ret



