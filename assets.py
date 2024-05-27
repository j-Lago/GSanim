
contrast_color_scale = 0.7

fonts = {
        'default': ('JetBrains Mono Medium', 12),
        'binds': ('JetBrains Mono Medium', 8),
        'fps': ('JetBrains Mono Medium', 14),
    }

cl = {'bg': '#ffffff',
      'airgap': '#ffffff',
      'outline': '#666666',
      # 'a': '#aa4422',
      # 'b': '#2244aa',
      # 'c': '#44aa22',
      'a': '#ff8419',
      'b': '#2a7db7',
      'c': '#00A859',
      's': '#773191',
      'x': '#FFB74D', #'#ff6347', #'#ff6699', #'#cc6633',
      'y': '#69C9D4', #'#0098DA', #'#7a7acc', #'#3366cc',
      'z': '#51CC90', #'#33a333', # '#3d995c', #'#33cc66',
      'r': '#FF6699',
      'l': '#51CC90',
      'g': '#51CC90',
      'capability': '#773191',
      't': '#FFB74D',
      'Fe': '#D5DBDB',
      'shaft': '#777777',
      'nan': '#000000',
      '_': '#000000',
      'I1': '#3e86f0',
      'I2': '#ae7bb5',
      'Tind': '#13ab45',
      'Tres': '#CB4335',
      'ws_cursor': '#9B59B6',
      'wr_cursor': '#777777',
      'grid': '#3d3d3d',
      'default_font': '#1C2833',
      'bind_font': '#849974',
      'delta': '#9B59B6',
      'Tturb': '#CB4335',
      }

w = 2
esp_r = 0.073
esp_sc = 0.78

s = 0.13          # shaft
k = (0.055, 0.055)   # keyway
ko = -k[1]*0.25

esprx = 96/350
espry = 173/350
esp_rc = espry
assets = {
    'stator_cutout':         {'shape': 'polygon',   'fill': cl['airgap'], 'stroke': '',            'width': w, 'coords': (-0.04,-0.67429,-0.04,-0.69143,-0.04,-0.69429,-0.042857,-0.7,-0.042857,-0.70286,-0.045714,-0.70571,-0.051429,-0.71143,-0.054286,-0.71429,-0.054286,-0.71429,-0.057143,-0.71714,-0.065714,-0.72571,-0.071429,-0.73143,-0.077143,-0.74,-0.08,-0.74857,-0.082857,-0.76,-0.085714,-0.76857,-0.085714,-0.78,-0.082857,-0.79714,-0.08,-0.81429,-0.071429,-0.82857,-0.06,-0.84,-0.048571,-0.85143,-0.034286,-0.85714,-0.017143,-0.86286,0,-0.86571,0.017143,-0.86286,0.034286,-0.85714,0.048571,-0.85143,0.06,-0.84,0.071429,-0.82857,0.08,-0.81429,0.082857,-0.79714,0.085714,-0.78,0.085714,-0.76857,0.082857,-0.76,0.08,-0.74857,0.077143,-0.74,0.071429,-0.73143,0.065714,-0.72571,0.057143,-0.71714,0.057143,-0.71714,0.054286,-0.71429,0.051429,-0.71143,0.045714,-0.70571,0.042857,-0.70286,0.042857,-0.7,0.04,-0.69429,0.04,-0.69143,0.04,-0.67429)},
    'stator_cutout_outline': {'shape': 'line',      'fill': '',           'stroke': cl['outline'], 'width': w, 'coords': (-0.04,-0.68571,-0.04,-0.69143,-0.04,-0.69429,-0.042857,-0.7,-0.042857,-0.70286,-0.045714,-0.70571,-0.051429,-0.71143,-0.054286,-0.71429,-0.054286,-0.71429,-0.057143,-0.71714,-0.065714,-0.72571,-0.071429,-0.73143,-0.077143,-0.74,-0.08,-0.74857,-0.082857,-0.76,-0.085714,-0.76857,-0.085714,-0.78,-0.082857,-0.79714,-0.08,-0.81429,-0.071429,-0.82857,-0.06,-0.84,-0.048571,-0.85143,-0.034286,-0.85714,-0.017143,-0.86286,0,-0.86571,0.017143,-0.86286,0.034286,-0.85714,0.048571,-0.85143,0.06,-0.84,0.071429,-0.82857,0.08,-0.81429,0.082857,-0.79714,0.085714,-0.78,0.085714,-0.76857,0.082857,-0.76,0.08,-0.74857,0.077143,-0.74,0.071429,-0.73143,0.065714,-0.72571,0.057143,-0.71714,0.057143,-0.71714,0.054286,-0.71429,0.051429,-0.71143,0.045714,-0.70571,0.042857,-0.70286,0.042857,-0.7,0.04,-0.69429,0.04,-0.69143,0.04,-0.68571)},
    'stator_inner':          {'shape': 'circle',    'fill': cl['airgap'], 'stroke': cl['outline'], 'width': w, 'coords': (0.0, 0.0, .6857)},
    'stator_outer':          {'shape': 'circle',    'fill': cl['Fe'],     'stroke': cl['outline'], 'width': w, 'coords': (0.0, 0.0, .98)},
    'rotor_outer':           {'shape': 'circle',    'fill': cl['Fe'],     'stroke': cl['outline'], 'width': w, 'coords': (0.0, 0.0, .66)},
    # 'rotor_cutout':          {'shape': 'polygon',   'fill': cl['airgap'], 'stroke': '',            'width': w, 'coords': (-0.04,-0.66571,-0.04,-0.65143,-0.04,-0.64857,-0.042857,-0.64286,-0.042857,-0.64,-0.045714,-0.63429,-0.051429,-0.63143,-0.054286,-0.62857,-0.054286,-0.62857,-0.057143,-0.62571,-0.065714,-0.61714,-0.071429,-0.60857,-0.077143,-0.60286,-0.08,-0.59143,-0.082857,-0.58286,-0.085714,-0.57143,-0.085714,-0.56286,-0.082857,-0.54571,-0.08,-0.52857,-0.071429,-0.51429,-0.06,-0.50286,-0.048571,-0.49143,-0.034286,-0.48286,-0.017143,-0.48,0,-0.47714,0.017143,-0.48,0.034286,-0.48286,0.048571,-0.49143,0.06,-0.50286,0.071429,-0.51429,0.08,-0.52857,0.082857,-0.54571,0.085714,-0.56286,0.085714,-0.57143,0.082857,-0.58286,0.08,-0.59143,0.077143,-0.60286,0.071429,-0.60857,0.065714,-0.61714,0.057143,-0.62571,0.057143,-0.62571,0.054286,-0.62857,0.051429,-0.63143,0.045714,-0.63429,0.042857,-0.64,0.042857,-0.64286,0.04,-0.64857,0.04,-0.65143,0.04,-0.66571)},
    'rotor_cutout':          {'shape': 'polygon',   'fill': cl['airgap'], 'stroke': '',            'width': w, 'coords': (-0.40286,-0.28571,-0.40571,-0.30286,-0.41143,-0.32,-0.42,-0.33143,-0.43143,-0.34571,-0.44571,-0.35429,-0.45429,-0.36,-0.46286,-0.36286,-0.47429,-0.36571,-0.48286,-0.36571,-0.49143,-0.36571,-0.50286,-0.36571,-0.51143,-0.36286,-0.51143,-0.36286,-0.51714,-0.36,-0.52286,-0.36,-0.52857,-0.35714,-0.53143,-0.35714,-0.53714,-0.35714,-0.54,-0.36,-0.54571,-0.36,-0.55714,-0.36857,-0.57429,-0.34286,-0.6,-0.29429,-0.62286,-0.24286,-0.64571,-0.18,-0.66571,-0.082857,-0.67143,0,-0.66286,0.094286,-0.64286,0.18571,-0.61143,0.27429,-0.58,0.33143,-0.56286,0.36,-0.54,0.36,-0.53714,0.35714,-0.53143,0.35714,-0.52857,0.35714,-0.52286,0.36,-0.51714,0.36,-0.51714,0.36,-0.51143,0.36286,-0.50286,0.36571,-0.49143,0.36571,-0.48286,0.36571,-0.47429,0.36571,-0.46286,0.36286,-0.45429,0.36,-0.44571,0.35429,-0.43143,0.34571,-0.42,0.33143,-0.41143,0.32,-0.40571,0.30286,-0.40286,0.28571)},
    # 'rotor_cutout_outline':  {'shape': 'line',      'fill': '',           'stroke': cl['outline'], 'width': w, 'coords': (-0.04,-0.65714,-0.04,-0.65143,-0.04,-0.64857,-0.042857,-0.64286,-0.042857,-0.64,-0.045714,-0.63429,-0.051429,-0.63143,-0.054286,-0.62857,-0.054286,-0.62857,-0.057143,-0.62571,-0.065714,-0.61714,-0.071429,-0.60857,-0.077143,-0.60286,-0.08,-0.59143,-0.082857,-0.58286,-0.085714,-0.57143,-0.085714,-0.56286,-0.082857,-0.54571,-0.08,-0.52857,-0.071429,-0.51429,-0.06,-0.50286,-0.048571,-0.49143,-0.034286,-0.48286,-0.017143,-0.48,0,-0.47714,0.017143,-0.48,0.034286,-0.48286,0.048571,-0.49143,0.06,-0.50286,0.071429,-0.51429,0.08,-0.52857,0.082857,-0.54571,0.085714,-0.56286,0.085714,-0.57143,0.082857,-0.58286,0.08,-0.59143,0.077143,-0.60286,0.071429,-0.60857,0.065714,-0.61714,0.057143,-0.62571,0.057143,-0.62571,0.054286,-0.62857,0.051429,-0.63143,0.045714,-0.63429,0.042857,-0.64,0.042857,-0.64286,0.04,-0.64857,0.04,-0.65143,0.04,-0.65714)},
    'rotor_cutout_outline':  {'shape': 'line',      'fill': '',           'stroke': cl['outline'], 'width': w, 'coords': (-0.55143,0.36571,-0.54571,0.36,-0.54,0.36,-0.53714,0.35714,-0.53143,0.35714,-0.52857,0.35714,-0.52286,0.36,-0.51714,0.36,-0.51714,0.36,-0.51143,0.36286,-0.50286,0.36571,-0.49143,0.36571,-0.48286,0.36571,-0.47429,0.36571,-0.46286,0.36286,-0.45429,0.36,-0.44571,0.35429,-0.43143,0.34571,-0.42,0.33143,-0.41143,0.32,-0.40571,0.30286,-0.40286,0.28571,-0.40286,-0.28571,-0.40571,-0.30286,-0.41143,-0.32,-0.42,-0.33143,-0.43143,-0.34571,-0.44571,-0.35429,-0.45429,-0.36,-0.46286,-0.36286,-0.47429,-0.36571,-0.48286,-0.36571,-0.49143,-0.36571,-0.50286,-0.36571,-0.51143,-0.36286,-0.51143,-0.36286,-0.51714,-0.36,-0.52286,-0.36,-0.52857,-0.35714,-0.53143,-0.35714,-0.53714,-0.35714,-0.54,-0.36,-0.54571,-0.36,-0.55143,-0.36571)},
    'stator_esp':            {'shape': 'circle',    'fill': '',           'stroke': '',            'width': w, 'coords': (0.0, esp_sc, esp_r)},
    'stator_esp_front':      {'shape': 'polygon',   'fill': '',           'stroke': '',            'width': w, 'stipple': '', 'state': 'hidden', 'coords': (-esp_r, esp_sc, -esp_r, -esp_sc, esp_r, -esp_sc, esp_r, esp_sc)},
    # 'rotor_esp':             {'shape': 'circle',    'fill': '',           'stroke': '',            'width': w, 'coords': (0.0, esp_rc, esp_r)},
    'rotor_esp':             {'shape': 'circle',    'fill': '',           'stroke': '',            'width': w, 'coords': (esprx, espry, esp_r)},
    'rotor_esp_front':       {'shape': 'polygon',   'fill': '',           'stroke': '',            'width': w, 'stipple': '', 'state': 'hidden', 'coords': (-esp_r, esp_rc, -esp_r, -esp_rc, esp_r, -esp_rc, esp_r, esp_rc)},
    'stator_axis':           {'shape': 'line',      'fill': '#ff0000',    'stroke': '#000000',     'width': w, 'state': 'hidden', 'dash': (20, 1), 'coords': (-0.98, 0.0, 0.98, 0.0)},
    'rotor_axis':            {'shape': 'line',      'fill': '#ff0000',    'stroke': '#000000',     'width': w, 'state': 'hidden', 'dash': (20, 1), 'coords': (-0.65, 0.0, 0.65, 0.0)},
    'arrow_test':            {'shape': 'line',      'fill': '#ff0000',    'stroke': '#000000',     'width': w, 'arrow': 'last', 'arrowshape': (18, 20, 6), 'coords': (-0.85, 0.0, 0.85, 0.0)},
    'quarter_flux':          {'shape': 'line',      'fill': '',           'stroke': cl['a'],       'width': 3*w, 'state': 'hidden', 'joinstyle': 'round', 'capstyle': 'round', 'arrow': 'last', 'arrowshape': (18, 20, 6), 'coords': (0,-0.92857,-0.04,-0.92857,-0.082857,-0.92857,-0.11714,-0.92571,-0.15143,-0.92286,-0.19429,-0.91429,-0.22286,-0.90857,-0.26286,-0.9,-0.30571,-0.88571,-0.34,-0.87429,-0.37714,-0.86,-0.42,-0.84,-0.46,-0.81714,-0.5,-0.79143,-0.53714,-0.76571,-0.57429,-0.73714,-0.60857,-0.71143,-0.62857,-0.69143,-0.65429,-0.66857,-0.68,-0.64,-0.70571,-0.61143,-0.72571,-0.58286,-0.74571,-0.55429,-0.76,-0.53143,-0.77143,-0.50571,-0.78,-0.48857,-0.78571,-0.46857,-0.79429,-0.45143,-0.8,-0.42857,-0.80286,-0.41143,-0.80571,-0.4,-0.80571,-0.38571,-0.80857,-0.37143,-0.80857,-0.35714,-0.80857,-0.34286,-0.80571,-0.32857,-0.80571,-0.31429,-0.80286,-0.30286,-0.8,-0.29143,-0.79714,-0.28,-0.79429,-0.26857,-0.78857,-0.25714,-0.78571,-0.24857,-0.78,-0.23714,-0.77429,-0.22286,-0.76571,-0.21143,-0.76,-0.2,-0.75429,-0.19143,-0.74571,-0.18286,-0.73714,-0.17429,-0.72571,-0.16571,-0.71429,-0.15714,-0.70286,-0.14857,-0.68286,-0.14286,-0.66571,-0.13429,-0.64,-0.12571,-0.61143,-0.12,-0.57429,-0.11429,-0.53143,-0.10857,-0.48571,-0.10286,-0.43429,-0.1,-0.38286,-0.094286,-0.30286,-0.091429,-0.25143,-0.088571,-0.2,-0.088571,-0.12286,-0.088571,-0.057143,-0.088571,0,-0.088571)},
    'shaft':                 {'shape': 'circle',    'fill': cl['shaft'],  'stroke': cl['outline'], 'width': w, 'coords': (0.0, 0.0, s)},
    'keyway':                {'shape': 'polygon',   'fill': cl['Fe'],     'stroke': '',            'width': w, 'coords': (-k[0], k[1]+s, -k[0], -k[1]+s, k[0], -k[1]+s, k[0], k[1]+s)},
    'keyway_outline':        {'shape': 'line',      'fill': '',           'stroke': cl['outline'], 'width': w, 'coords': (-k[0], s+ko, -k[0], -k[1]+s, k[0], -k[1]+s, k[0], s+ko)},
    'mount':                 {'shape': 'polygon',   'fill': cl['Fe'],     'stroke': cl['outline'], 'width': w, 'coords': (.52, -.65, .7, -.98, .35, -.98, .25, -.7)},
    'vec':                   {'shape': 'line',      'fill': '#000000',    'stroke': '#000000',     'width': w, 'joinstyle': 'round', 'capstyle': 'round', 'arrow': 'last', 'arrowshape': (18, 20, 6), 'coords': (0.0, 0.0, 0.4, 0.0)},
    'in_s':                  {'shape': 'line',      'fill': '#000000',    'stroke': '#000000',     'width': 2*w, 'joinstyle': 'round', 'capstyle': 'round', 'coords': (-0.04, -0.04+esp_sc, 0.04, 0.04+esp_sc, 0.0, 0.0+esp_sc, -0.04, 0.04+esp_sc, 0.04, -0.04+esp_sc), 'anchor': (0.0, 0.0+esp_sc)},
    'out_s':                 {'shape': 'circle',    'fill': '#000000',    'stroke': '#000000',     'width': 2*w, 'coords': (0, esp_sc, 0.012), 'anchor': (0, esp_sc)},
    'in_r':                  {'shape': 'line',      'fill': '#000000',    'stroke': '#000000',     'width': 2*w, 'joinstyle': 'round', 'capstyle': 'round', 'coords': (-0.04+esprx, -0.04+espry, 0.04+esprx, 0.04+espry, 0.0+esprx, 0.0+espry, -0.04+esprx, 0.04+espry, 0.04+esprx, -0.04+espry)},
    'out_r':                 {'shape': 'circle',    'fill': '#000000',    'stroke': '#000000',     'width': 2*w, 'coords': (0+esprx, espry, 0.012)},
}



binds_message = r"""
grid angular speed: <up><down>
rotor speed: <left><right>
stator speed: <+><->

stator magnetic axis: <[>
rotor magnetic axis: <]>

show rotor: <r>
show stator: <s>
show rotor field: <>
show stator field: <f>

reset time and stop: <esc>
reset time: <0>
run/stop: <space>

frame delay: </><*>
"""