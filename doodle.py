from MITsim import MITsim
from colorsys import hls_to_rgb, rgb_to_hls
from math import pi
# from collections import OrderedDict
import numpy as np
import matplotlib.pyplot as plt
from transformations import hex_lerp, hex_to_rgb, hex_bezier
from GSsim import GSsim

def color_bezier():
    c0, c1, c2 = '#ff6600', '#aaffaa', '#0066ff'
    fig, ax = plt.subplots(1, 1)
    cls = []
    for i in range(11):
        cls.append(hex_bezier(c0, c1, c2, 0.1*i))
        ax.plot(0, 0.1*i, 's', markersize=30, color=hex_to_rgb(cls[i]))
    plt.show()

def gs():

    ger = GSsim(1, 60, .00265258238489, .0)
    ths = np.linspace(0.0, 2*pi, 100, dtype=float)
    Tes = np.empty_like(ths)
    for k in range(len(ths)):
        E, I, Te, Pe, Sout = ger.solve(1.0, ths[k])
        Tes[k] = Te

    plt.plot(ths, Tes)
    plt.plot((0, 2*pi), (1, 1), '-.k')
    plt.show()


    ret = ger.solve(1.0, pi/2, return_type=dict)
    print(ret, type(ret))



if __name__ == '__main__':
    # color_bezier()
    gs()