from MITsim import MITsim
from colorsys import hls_to_rgb, rgb_to_hls
from math import pi
# from collections import OrderedDict
import numpy as np
import matplotlib.pyplot as plt
from transformations import hex_lerp, hex_to_rgb, hex_bezier

def main():

    c0, c1, c2 = '#ff6600', '#aaffaa', '#0066ff'

    fig, ax = plt.subplots(1, 1)
    cls = []
    for i in range(11):
        cls.append(hex_bezier(c0, c1, c2, 0.1*i))

        ax.plot(0, 0.1*i, 's', markersize=30, color=hex_to_rgb(cls[i]))
    plt.show()



if __name__ == '__main__':
    main()