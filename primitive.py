from __future__ import annotations
from NormCanvas import NormCanvas
from transformations import Coords, Point, Numeric, denorm_coords
from math import sqrt
import transformations
from typing import Literal, Any
from copy import copy, deepcopy

from types import FunctionType
from copy import copy, deepcopy



def recursive_parent_visible(obj, parent_visibility, node_visibility=None):
    if isinstance(obj, PrimitivesGroup):
        obj._parent_visibility = parent_visibility
        if node_visibility is not None:
            obj._visibility = node_visibility

        for child in obj.nodes:
            recursive_parent_visible(child, parent_visibility and obj._visibility, node_visibility)
    else:
        obj.visible = parent_visibility

class Primitive:
    def __init__(self, canvas: NormCanvas,
                 shape: Literal['line', 'polygon', 'rectangle', 'square', 'circle', 'oval', 'arc', 'text'],
                 coords: Coords,
                 transforms: tuple[FunctionType, tuple] | list[tuple[FunctionType, tuple]] | tuple[tuple[FunctionType, tuple]] | Any | None = None,
                 name='unnamed',
                 anchor=(0.0, 0.0),
                 **kwargs):
        """
        """
        self._canvas = canvas
        self._handle = None
        self.name = name


        if transforms:
            if isinstance(transforms[0], FunctionType):
                transforms = [transforms]
            for transform_args in transforms:
                if len(transform_args) == 2:
                    transform, args  = transform_args
                    if isinstance(args, float | int):
                        coords = transform(coords, args)
                        anchor = transform(anchor, args)
                    elif isinstance(args, tuple):
                        if isinstance(args[0], float | int):
                            coords = transform(coords, args)
                            anchor = transform(anchor, args)
                        else:
                            coords = transform(coords, *args)
                            anchor = transform(anchor, *args)
                    elif isinstance(args, dict):
                        coords = transform(coords, **args)
                        anchor = transform(anchor, **args)

                else:
                    coords = transform_args[0](coords)
                    anchor = transform_args[0](anchor)


        self.coords = coords
        self.anchor = anchor
        self.original_coords = coords
        self.original_anchor = anchor
        self.shape = shape

        self._create(**kwargs)


    @staticmethod
    def merge(primitives_list, merge_into_id: int = 0):
        coords = []
        merged = primitives_list[merge_into_id]

        first_shape = merged.shape
        for p in primitives_list:
            if p.shape != first_shape:
                raise ValueError(f"só é suportado combinar primitivas de mesmo 'shape': {first_shape=} != {p.shape=}")
            coords.append(p.coords)

        merged.coords = tuple(v for pt in coords for v in pt)

        return merged

    @property
    def arrow(self) -> str:
        return self._canvas.itemcget(self._handle, 'arrow')

    @arrow.setter
    def arrow(self, arrow):
        self._canvas.itemconfig(self._handle, arrow=arrow)

    @property
    def arrowshape(self) -> str:
        return self._canvas.itemcget(self._handle, 'arrowshape')

    @arrowshape.setter
    def arrowshape(self, arrowshape):
        self._canvas.itemconfig(self._handle, arrowshape=arrowshape)

    @property
    def fill(self) -> str:
        return self._canvas.itemcget(self._handle, 'fill')

    @fill.setter
    def fill(self, fill):
        self._canvas.itemconfig(self._handle, fill=fill)

    @property
    def stroke(self) -> str:
        key = 'outline' if self.shape != 'line' else 'fill'
        return self._canvas.itemcget(self._handle, key)

    @stroke.setter
    def stroke(self, stroke):
        if self.shape != 'line':
            self._canvas.itemconfig(self._handle, outline=stroke)
        else:
            self._canvas.itemconfig(self._handle, fill=stroke)

    @property
    def width(self) -> str:
        return self._canvas.itemcget(self._handle, 'width')

    @width.setter
    def width(self, width):
        self._canvas.itemconfig(self._handle, width=width)

    @property
    def visible(self) -> str:
        return self._canvas.itemcget(self._handle, 'state') != 'hidden'

    @visible.setter
    def visible(self, visible):
        self._canvas.itemconfig(self._handle, state='normal' if visible else 'hidden')


    @property
    def stipple(self) -> str:
        return self._canvas.itemcget(self._handle, 'stipple')

    @stipple.setter
    def stipple(self, stipple):
        self._canvas.itemconfig(self._handle, stipple=stipple)

    def reset_transformations(self):
        self.coords = self.original_coords
        self.anchor = self.original_anchor

    def reverse(self, from_original: bool = False):
        coords = self.original_coords if from_original else self.coords
        anchor = self.original_anchor if from_original else self.anchor

        self.anchor = transformations.reverse(anchor)
        self.coords = transformations.reverse(coords)
        return self

    def rotate(self, angle: Numeric, center: Point = (0, 0), from_original: bool = False):
        coords = self.original_coords if from_original else self.coords
        anchor = self.original_anchor if from_original else self.anchor

        self.anchor = transformations.rotate(anchor, angle, center)
        self.coords = transformations.rotate(coords, angle, center)
        return self

    def translate(self, offset: Point, from_original: bool = False):
        coords = self.original_coords if from_original else self.coords
        anchor = self.original_anchor if from_original else self.anchor

        self.anchor = transformations.translate(anchor, offset)
        self.coords = transformations.translate(coords, offset)
        return self

    def scale(self, factor: Numeric | Point, center: Point = (0, 0), from_original: bool = False):
        coords = self.original_coords if from_original else self.coords
        anchor = self.original_anchor if from_original else self.anchor

        self.anchor = transformations.scale(anchor, factor, center)
        self.coords = transformations.scale(coords, factor, center)
        return self

    def _create(self, dont_draw=False, **kwargs):
        # print('... >> ', kwargs)
        kwargs['outline'] = kwargs.pop('stroke', '')
        match self.shape:
            case 'line':
                kwargs['fill'] = kwargs.pop('outline', kwargs.pop('fill', ''))
                self._handle = self._canvas.create_line(self.denorm_coords(), **kwargs)
            case 'polygon':
                self._handle = self._canvas.create_polygon(self.denorm_coords(), **kwargs)
            case 'rectangle':
                self._handle = self._canvas.create_rectangle(self.denorm_coords(), **kwargs)
            case 'oval':
                raise ValueError(f"'primitive_type' ({self.shape}) inválido")
            case 'circle':
                self._handle = self._canvas.create_oval(self._denorm_coords_circle_to_oval(), **kwargs)
            case 'text':
                # kwargs['fill'] = kwargs.pop('outline', kwargs.pop('fill', ''))
                kwargs.pop('outline')
                kwargs['text'] = kwargs.pop('text', self.name)
                self._handle = self._canvas.create_text(self.denorm_coords(), **kwargs)
            case 'arc':
                raise ValueError(f"'primitive_type' ({self.shape}) inválido")
            case _:
                raise ValueError(f"'primitive_type' ({self.shape}) inválido")

        if not dont_draw:
            self.draw()

    def delete(self):
        self._canvas.delete(self._handle)
        del self

    def draw(self,
             reset_transforms_to_original_after_draw: bool = True,
             consolidate_transforms_to_original: bool = False,
             **kwargs):

        self._canvas.coords(self._handle, self.denorm_coords())
        self._canvas.itemconfig(self._handle, **kwargs)

        if consolidate_transforms_to_original:
            self.original_coords = self.coords
            self.original_anchor = self.anchor
        else:
            if reset_transforms_to_original_after_draw:
                self.coords = self.original_coords
                self.anchor = self.original_anchor



    def denorm_coords(self) -> Coords:
        if hasattr(self, 'shape'):
            if self.shape == 'circle' and len(self.coords) == 3:
                ret = self._denorm_coords_circle_to_oval()
                return ret

        return denorm_coords(self._canvas, self.coords)

    def _denorm_coords_circle_to_oval(self) -> Coords:
        oval_coords = transformations.coords_circle_to_oval(self.coords)
        return denorm_coords(self._canvas, oval_coords)


class PrimitivesGroup:
    def __init__(self, name: str, prims: list[Primitive] | PrimitivesGroup | list[PrimitivesGroup] = [], visibility: bool = True):
        self.name = name
        self.nodes = prims if isinstance(prims, list) else [prims]
        self._visibility = True  # use .visible para propagar aos subgrupos abaixo
        self._parent_visibility = True

    def append(self, item):
        self.nodes.append(item)

    @property
    def keys(self):
        return tuple(prim.name for prim in self.nodes)

    def get_index(self, key):
        return tuple(id for id, prim in enumerate(self.nodes) if prim.name == key)

    def filter_matching_keys(self, *keys):
        founds_child = []
        keys_copy = list(keys)

        if isinstance(keys_copy, str):
            keys_copy = [keys_copy]

        if len(keys_copy) > 0:
            root_key = keys_copy.pop(0)
            founds_root = self.filter_single_key(root_key)

            if len(keys_copy) > 0:
                for found in founds_root:
                    founds2 = found.filter_matching_keys(*keys_copy)
                    for found_child in founds2:
                        founds_child.append(found_child)
            else:
                return founds_root
        return founds_child


    def filter_single_key(self, key):
        ret = []
        for node in self.nodes:
            if isinstance(node, PrimitivesGroup):
                if node.name == key:
                    ret.append(node)
                founds = node.filter_single_key(key)
                for found in founds:
                    ret.append(found)

        return ret

    def __delitem__(self, key):
        for i in self.get_index(key):
            self[i].delete_descendant_nodes()
            del self.nodes[i]

    def delete_descendant_nodes(self):
        for node in self.nodes:
            if isinstance(node, PrimitivesGroup):
                node.delete_descendant_nodes()
            else:
                node.delete()
        self.nodes = []





    def __getitem__(self, key):

        if isinstance(key, int):
            return self.nodes[key]

        if not isinstance(key, str):
            raise ValueError("'id' deve ser int ou str" )

        for k, prim in enumerate(self.nodes):
            if prim.name == key:
                return prim

        raise ValueError(f"o grupo não possui um elemento de nome '{key}'")

    def __setitem__(self, key, value):

        if not isinstance(key, str):
            raise ValueError("'key' deve do tipo str")

        if key not in self.keys:
            if isinstance(value, PrimitivesGroup):
                self.nodes.append(value)
                return

            if isinstance(value, Primitive):
                self.nodes.append(PrimitivesGroup(key, [value]))
                return

            if isinstance(value, list):
                self.nodes.append(PrimitivesGroup(key, value))
                return

            raise ValueError("'value' deve do tipo PrimitivesGroup ou Primitive ou list")

        else:
            if isinstance(value, PrimitivesGroup):
                for id in self.get_index(key):
                    self.nodes[id] = value
                return

            if isinstance(value, Primitive):
                for id in self.get_index(key):
                    self.nodes[id] = PrimitivesGroup(key, [value])
                return

            if isinstance(value, list):
                for id in self.get_index(key):
                    self.nodes[id] = PrimitivesGroup(key, value)
                return

            raise ValueError("'value' deve do tipo PrimitivesGroup ou Primitive")
        raise ValueError('assert')


    @property
    def visible(self):
        return self._visibility

    @visible.setter
    def visible(self, visiblitity):
        self._visibility = visiblitity
        for child in self.nodes:
            recursive_parent_visible(child, parent_visibility=self._visibility)

    @property
    def parent_visible(self):
        return self._parent_visibility

    def toggle_visible(self):
        self.visible = not self.visible


    def stroke(self, stroke):
        for child in self.nodes:
            child.stroke = stroke
    stroke = property(None, stroke)

    def fill(self, fill):
        for child in self.nodes:
            child.fill = fill
    fill = property(None, fill)

    def width(self, width):
        for child in self.nodes:
            child.width = width
    width = property(None, width)

    def stipple(self, stipple):
        for child in self.nodes:
            child.stipple = stipple
    stipple = property(None, stipple)

    def rotate(self, *args, **kwargs):
        for child in self.nodes:
            child.rotate( *args, **kwargs)

    def translate(self, *args, **kwargs):
        for child in self.nodes:
            child.translate( *args, **kwargs)

    def scale(self, *args, **kwargs):
        for child in self.nodes:
            child.scale( *args, **kwargs)

    def reverse(self, *args, **kwargs):
        for child in self.nodes:
            child.reverse( *args, **kwargs)

    def draw(self, level=0, parent_visible=True, *args, **kwargs):
        for child in self.nodes:
            child.draw(*args, **kwargs)

    def print_tree(self, print_leafs=True, level=0):
        class style:
            MAGENTA = '\033[95m'
            BLUE = '\033[94m'
            CYAN = '\033[96m'
            GREEN = '\033[92m'
            YELLOW = '\033[93m'
            RED = '\033[91m'
            GRAY = '\033[0m'
            BOLD = '\033[1m'
            RESET = '\033[m'
            UNDERLINE = '\033[4m'
            NODE = BLUE
            LEAF = CYAN
            iNODE = RED
            iLEAF = MAGENTA

        class symbol:
            CROSS = style.RED + '\u2718'
            CHECK = style.GREEN + '\u2714'

        node_sty = style.NODE if self.visible and self.parent_visible else style.iNODE
        print(f"{'.  ' * level}' + node_sty + f'↳ {self.name}[{len(self.nodes)}] ({symbol.CHECK if self.parent_visible else symbol.CROSS}{symbol.CHECK if self.visible else symbol.CROSS}' + node_sty + f'):'  + f' at {hex(id(self))}" + style.RESET)
        for prim in self.nodes:
            if isinstance(prim, PrimitivesGroup):
                prim.print_tree(print_leafs=print_leafs, level=level+1)
            elif print_leafs:
                leaf_sty = style.LEAF if prim.visible else style.iLEAF
                print(f"{'.  ' * (level + 1)}' + node_sty + '↳  f' + leaf_sty + f'{prim.name} ({symbol.CHECK if prim.visible else symbol.CROSS}' + leaf_sty + f'): {prim.shape}  ⇣{prim._handle}" + style.RESET)


def collision_circle_point(circle: Primitive, point: tuple[float, float]) -> bool:
    c = complex(*circle.coords[0:2])
    r = circle.coords[2]
    p = complex(*point)
    return abs(p-c) < r



