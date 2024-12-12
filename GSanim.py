import tkinter as tk
from cmath import polar, rect, phase
from functools import partial

import matplotlib.pyplot as plt
import numpy as np
from tkinter import messagebox
from math import sin, cos, pi, atan2, sqrt, fabs
from NormCanvas import NormCanvas, BoolVar
from primitive import Primitive, PrimitivesGroup, collision_circle_point
from transformations import norm_coords, denorm_coords, translate, rotate, scale, reverse, rgb_to_hex, hex_to_rgb, scale_hsl, set_hsl, clip, CircularDict, hex_bezier, scale_rgb, denorm_coords
import time
from assets import assets, cl, binds_message, contrast_color_scale
from animation import Animation, PlotLimits, AxisLimits
from collections import deque
from statistics import mean
from typing import Literal
from machine import SynchronousGenerator
from custom_graphics import gs_draw, circular_pattern, IndexFilter3ph, create_stator
from threading import Thread, Event
import globals

def plot_thread(event_redraw: Event, figs):
    while True:
        event_redraw.wait()
        for fig in figs:
            fig.axes.clear()
            fig.canvas.draw()
        event_redraw.clear()

# def plot_thread_fig0(event_redraw: Event, fig):
#     while True:
#         event_redraw.wait(.2)
#         fig.canvas.draw()
#         event_redraw.clear()
#
# def plot_thread_fig1(event_redraw: Event, fig):
#     while True:
#         event_redraw.wait(.2)
#         fig.canvas.draw()
#         event_redraw.clear()


class CustomAnim(Animation):
    def __init__(self, canvas: NormCanvas, widgets: dict = None):
        super().__init__(canvas, frame_delay=0)

        self.plt_lines = {}
        self.widgets = widgets

        self._f_ref_inc = None
        self._f_rot_inc = None
        self._f_grid_inc = None

        self.B_liq = None
        self.B_arm = None
        self.B_field = None
        self.v_grid = None
        self.i_field = None
        self.f_grid = None
        self.f_ref = None
        self.f_rot = None

        self.th_grid = None
        self.th_ref = None
        self.th_rot = None

        self.t = None
        self.time_factor = None
        self.reset_time()

        self.inertia = 0.0002

        self.run = True
        self._previous_run = True

        self.en_sim_inertia = BoolVar(True)

        # controles de visualização
        self.en_stator_field_lines = BoolVar(False)
        self.en_stator_field_vec = BoolVar(True)
        self.en_rotor_field_lines = BoolVar(False)
        self.en_rotor_field_vec = BoolVar(True)
        self.en_stator_coil_front = BoolVar(False)
        self.en_rotor_coil_front = BoolVar(False)
        self.sel_mount = CircularDict({'normal': True, 'hidden': False})

        # para média móvel da exibição do FPS
        self.dt_filter_buffer = deque(maxlen=100)
        self.dc_filter_buffer = deque(maxlen=100)
        self.plot_downsample_factor = 3

        self.sel_reference_frame = tk.StringVar(value='free')
        self.sel_dynamic_color = CircularDict({'3phase': False, 'amplitude': True})

        self.sel_stator_field = CircularDict({'abc': 0, 'abcs': 1, 'a': 2, 'b': 3, 'c': 4, 's': 5, 'l': 6, 'sl': 7})
        self.sel_stator_field.key = 'sl'

        self.sel_rotor_field = CircularDict({'r': 0})
        self.sel_rotor_field.key = 'r'

        self.dumping = CircularDict({'high': 60, 'normal': 30, 'low': 5, 'off': 0})
        self.dumping.name = 'damping factor'

        self.sel_fig1 = CircularDict({'': (-0.5, 3.5)})  #  'atributo': ylim
        self.sel_fig0 = CircularDict({'V_abc': 1.5,
                                      'I_abc': 1.5,
                                      'E_abc': 1.5,
                                      'VE_lr': 1.5,
                                      'f_grt': (30, 90)
                                      }) #,PlotLimits(ymin=30, ymax=90, twinx= PlotLimits(ymin=-30, ymax=110)) })  # 'atributo': ylim

        self.sel_stator_turns = CircularDict({'simp': (2, 3), '4': (4, 3), '6': (6, 3), '8': (8, 3)})  # versão do estator com n espiras por fase
        self.sel_rotor_turns = CircularDict({'simp': (2, 3), '4': (4, 3), '6': (6, 3)})  # versão do estator com n espiras por fase
        self.sel_coil_opacity = CircularDict({'gray75': 1, 'gray50': 2, 'gray25': 3})  # value não utilizado

        self.sel_fs_unit = CircularDict({'Hz': 1.0, 'rad/s': 2 * pi, 'rpm': 60})  # 'um': fator de conversão
        self.sel_fr_unit = CircularDict({'Hz': 1.0, 'rad/s': 2 * pi, 'rpm': 60})  # 'um': fator de conversão
        self.sel_fg_unit = CircularDict({'Hz': 1.0, 'rad/s': 2 * pi, 'rpm': 60})  # 'um': fator de conversão
        self.sel_Pout_unit = CircularDict({'pu': 1.0})  # 'um': fator de conversão
        self.sel_Vg_unit = CircularDict({'pu': 1.0})  # 'um': fator de conversão

        self.gs = SynchronousGenerator(1.0, 60.0, 1.8, 0.0)
        self.Tturb = .0
        self.Tturb0 = 0.01


        # para plots
        self.ax_npt_fig0 = 100
        self.ax_npt_fig1 = 100
        self.plt_t = deque(maxlen=self.ax_npt_fig0)
        self.plt_y = (deque(maxlen=self.ax_npt_fig0), deque(maxlen=self.ax_npt_fig0), deque(maxlen=self.ax_npt_fig0))
        self.plt_yy = deque(maxlen=self.ax_npt_fig0)
        # plt.rcParams['font.family'] = 'sans-serif'
        # plt.rcParams['font.sans-serif'] = 'corbel'

        self.create_fig0()
        self.create_fig1()
        self.create_fig2()

        self.prims = PrimitivesGroup('root', [])
        self.create_primis()




        self.binds()

        self.thread_figs_redraw = Event()
        self.thread_figs = Thread(target=plot_thread, args=(self.thread_figs_redraw, self.widgets['figs']), daemon=True)
        self.thread_figs.start()
        # self.thread_fig0_redraw = Event()
        # self.thread_fig0 = Thread(target=plot_thread_fig0, args=(self.thread_fig0_redraw, self.widgets['figs'][0]), daemon=True)
        # self.thread_fig1_redraw = Event()
        # self.thread_fig1 = Thread(target=plot_thread_fig1, args=(self.thread_fig1_redraw, self.widgets['figs'][1]), daemon=True)
        # self.thread_fig0.start()
        # self.thread_fig1.start()


    @property
    def mouse_norm_coords(self):
        return norm_coords(canvas=self.canvas, coords=(self.canvas.winfo_pointerx() - self.canvas.winfo_rootx(),
                                                       self.canvas.winfo_pointery() - self.canvas.winfo_rooty()))


    def create_fig0(self):

        ax = self.widgets['figs'][0].axes[0]
        axt = ax.twinx()
        self.widgets['figs'][0].axes.append(axt)

        marker_size = 8

        ax.axhline(0, linestyle='--', color=cl['grid'], linewidth=0.8)
        self.plt_lines['fig0_hline'] = ax.axhline(0, linestyle='--', color=cl['grid'], linewidth=0.8)
        self.plt_lines['fig0t_hline'] = axt.axhline(np.nan, linestyle='--', color=cl['t'], linewidth=0.8)


        self.plt_t.extend(np.linspace(-0.04, 0.00, self.ax_npt_fig0))
        self.invalidate_fig0_data(self.time_factor)

        ax.set_ylim(-1.2, 1.2)
        ax.set_xticks([])

        for i, ph in enumerate('abc'):
            self.plt_lines[ph] = (ax.plot(self.plt_t, self.plt_y[i], color=cl[ph], lw=2))[0]
            self.plt_lines[ph+'_marker'] = (ax.plot((0.0,), (0.0,), color=cl[ph], marker='o', markersize=marker_size))[0]



        self.plt_lines['twinx'] = (axt.plot(self.plt_t, self.plt_y[i], color=cl['c'], lw=2))[0]
        self.plt_lines['twinx_marker'] = (axt.plot((0.0,), (float('nan'),), color=cl['c'], marker='o', markersize=marker_size))[0]

    def create_fig1(self):
        marker_size = 8
        cursor_lw = 1.5
        nmax = 3800

        ax = self.widgets['figs'][1].axes[0]
        ax.axhline(0, linestyle='--', color=cl['grid'], linewidth=0.8)
        ax.axvline(0, linestyle='--', color=cl['grid'], linewidth=0.8)


        self.plt_lines['Tturb'] = ax.axhline(0, linestyle='-', color=cl['Tturb'], lw=cursor_lw)
        self.plt_lines['delta'] = ax.axvline(0, linestyle='--', color=cl['t'], lw=cursor_lw)
        self.plt_lines['Tind'] = (ax.plot(0, 0, color=cl['Tind'], lw=2))[0]
        self.plt_lines['Ix'] = (ax.plot(0, 0, color=cl['I1'], lw=2))[0]
        # self.plt_lines['Pturb_marker'] = (ax.plot(0, 0, color=cl['Tres'], marker='o', markersize=marker_size, lw=2))[0]
        self.plt_lines['Pout_marker'] = (ax.plot(0,0, color=cl['Tind'], marker='o', markersize=marker_size, lw=2))[0]
        ax.set_xlim(-pi*0.1, pi*1.1)
        ax.set_ylim(self.sel_fig1.value[0], self.sel_fig1.value[1])
        ax.set_ylabel('P')
        ax.set_xlabel(r'$\delta$')

    def create_fig2(self):
        # todo: criar curva de capabilidade com base no modelo
        marker_size = 8
        cursor_lw = 1.5
        nmax = 3800

        ax = self.widgets['figs'][1].axes[1]
        ax.axhline(0, linestyle='--', color=cl['grid'], linewidth=0.8)

        npt = 100
        Slim = 1.1
        F0 = -self.gs.Xs
        Fmin = 0.9
        Fmax = 2.6
        Pmax = 1.0


        ths = np.linspace(-pi/2, pi/2, npt)
        slim = np.array(list(rect(Slim, th) for th in ths))
        sfmin = np.array(list(rect(Fmin, th) + complex(0, F0) for th in ths))
        sfmax = np.array(list(rect(Fmax, th) + complex(0, F0) for th in ths))
        width = 1.2
        linestyle = '-.'

        ax.axvline(0, linestyle=linestyle, color=cl['a'], linewidth=width)
        ax.axvline(Pmax, linestyle=linestyle, color=cl['a'], linewidth=width)
        self.plt_lines['Slim'] = (ax.plot(slim.real, slim.imag, color=cl['s'], lw=width, linestyle=linestyle))[0]
        self.plt_lines['Fmin'] = (ax.plot(sfmin.real, sfmin.imag, color=cl['r'], lw=width, linestyle=linestyle))[0]
        self.plt_lines['Fmax'] = (ax.plot(sfmax.real, sfmax.imag, color=cl['r'], lw=width, linestyle=linestyle))[0]
        self.plt_lines['PQ_marker'] = (ax.plot(0,0, color=cl['Tind'], marker='o', markersize=marker_size, lw=2))[0]



        ax.set_xlim(-0.2, 1.4)
        ax.set_xticks(np.linspace(0,1,3))
        ax.set_ylim(-1.2, 1.2)
        ax.set_yticks(np.linspace(-1,1,5))

        ax.set_aspect('equal')
        ax.set_ylabel('Q')
        ax.set_xlabel('P')
        # self.widgets['figs'][1].tight_layout()


    def refresh(self, _, dt, frame_count):
        self.update_fps_info(dt)
        redraw_plt = frame_count % self.plot_downsample_factor == 0

        if self.run and self._previous_run:
            dt /= self.time_factor     # time scaled dt for animation
        else:
            dt = 0.0
        self._previous_run = self.run

        sim = self.gs.solve(self.v_grid, self.i_field, self.th_rot - self.th_grid, return_type=dict)

        if redraw_plt:
            self.update_fig1(sim)
            self.update_fig2(sim)

        alpha = 2 * pi / 3
        th = self.th_grid - self.th_ref
        V_abc = tuple(abs(sim['V']) * sin(th - i * alpha) for i in range(3))
        E_abc = tuple(abs(sim['E']) * sin(th + phase(sim['E']) - i * alpha) for i in range(3))
        I_abc = tuple(abs(sim['I']) * sin(th + phase(sim['I']) - i * alpha) for i in range(3))
        VE_lr = (np.nan, abs(sim['V']) * sin(th), abs(sim['E']) * sin(th+self.delta))

        # print(V_abc)

        sim = sim | {'V_abc': V_abc, 'E_abc': E_abc, 'I_abc': I_abc, 'If': self.i_field, 'VE_lr': VE_lr, 'test': VE_lr}


        if redraw_plt and self.run:
            self.update_fig0(sim)

        self.update_fields(sim)

        self.prims['rotor'].rotate(self.th_rot)
        self.prims['stator'].rotate(self.th_ref)

        self.prims['stator']['core']['mount'].visible = self.sel_mount.value and self.prims['stator']['core']['mount'].parent_visible
        # self.prims['rotor'].visible = self.select_part.key in ('rotor', 'all')
        # self.prims['stator'].visible = self.select_part.key in ('stator', 'all')

        if self.sel_dynamic_color.value:
            self.prims['stator']['coil_front'].visible = False
            self.prims['rotor']['coil_front'].visible = False
            self.color_stator_coils(sim['I_abc'])
            self.color_rotor_coils(sim['Ir'])
        else:
            self.prims['stator']['coil_front'].visible = self.en_stator_coil_front and self.prims['stator'].visible
            self.prims['rotor']['coil_front'].visible = self.en_rotor_coil_front and self.prims['rotor'].visible


        self.prims.draw()
        if redraw_plt:
            if not self.thread_figs_redraw.is_set():
                self.thread_figs_redraw.set()
            for fig in self.widgets['figs']:
                fig.canvas.flush_events()

        self.process_incs()
        self.process_inertia(dt, sim)
        th_ref_inc = 0.0
        match self.sel_reference_frame.get():
            case 'free':
                pass
            case 'stator':
                self._f_ref_inc -= self.f_ref
                th_ref_inc = -self.th_ref
            case 'rotor':
                self._f_ref_inc -= self.f_rot
                th_ref_inc = - phase(self.B_field)- self.th_ref
            case 'field':
                self._f_ref_inc -= self.f_grid
                th_ref_inc = - phase(self.B_liq)- self.th_ref
            case _:
                raise ValueError('assert')
        self.process_incs()


        if self.run:
            self.th_ref = (self.th_ref + dt * self.f_ref * 2 * pi) % (2 * pi) + th_ref_inc
            self.th_rot = (self.th_rot + dt * self.f_rot * 2 * pi) % (2 * pi) + th_ref_inc
            self.th_grid = (self.th_grid + dt * self.f_grid * 2 * pi) % (2 * pi) + th_ref_inc
            self.t += dt

        self.update_info(sim)

    def process_inertia(self, dt, sim):
        if self.en_sim_inertia:
            DT = (self.Tturb - sim['Te'])
            self.f_rot += DT * 377/self.inertia * dt - (self.f_rot-self.f_grid)*2*pi* self.dumping.value * dt
            self._f_rot_inc = 0.0
            # print(self.dumping.key, self.dumping.value)
        else:
            self.f_rot += self._f_rot_inc

    def process_incs(self):
        if self.run:
            self.f_rot += self._f_rot_inc
            self._f_rot_inc = 0.0

            self.f_grid+= self._f_grid_inc
            self._f_grid_inc = 0.0


            self.f_ref += self._f_ref_inc
            self.f_grid += self._f_ref_inc
            self.f_rot += self._f_ref_inc
            self._f_ref_inc = 0.0

    # ------------------------------- end of refresh -------------------------------------

    def color_stator_coils(self, I1_abc):
        if self.sel_dynamic_color.value:
            for i, ph in enumerate('abc'):
                amp = clip(I1_abc[i] * 0.33, -1.0, 1.0)
                tp = fabs(amp)
                tn = fabs(-amp)
                clp = hex_bezier('#ff6600', '#aaffaa', '#0066ff', amp * 0.5 + 0.5)
                cln = hex_bezier('#ff6600', '#aaffaa', '#0066ff', -amp * 0.5 + 0.5)

                if amp < 0:
                    clp, cln = cln, clp
                    tp, tn = tn, tp

                self.prims['stator']['coil']['in'][ph].fill = clp
                self.prims['stator']['coil']['in'][ph].stroke = clp
                self.prims['stator']['coil']['out'][ph].fill = cln
                self.prims['stator']['coil']['out'][ph].stroke = cln

                self.prims['stator']['current']['in'][ph].stroke = scale_rgb(clp, contrast_color_scale)
                self.prims['stator']['current']['in'][ph].fill = scale_rgb(clp, contrast_color_scale)
                self.prims['stator']['current']['out'][ph].stroke = scale_rgb(cln, contrast_color_scale)
                self.prims['stator']['current']['out'][ph].fill = scale_rgb(cln, contrast_color_scale)

                for j, _ in enumerate(self.prims['stator']['current']['in'][ph]):
                    self.prims['stator']['current']['in'][ph][j].scale(tp, center=self.prims['stator']['current']['in'][ph][j].anchor)
                    self.prims['stator']['current']['out'][ph][j].scale(tn, center=self.prims['stator']['current']['out'][ph][j].anchor)

                if amp < 0:
                    self.prims['stator']['current']['in'][ph].rotate(pi)
                    self.prims['stator']['current']['out'][ph].rotate(pi)
                    self.prims['stator']['coil']['in'][ph].rotate(pi)
                    self.prims['stator']['coil']['out'][ph].rotate(pi)


    def color_rotor_coils(self, Ir_xyz):
        for i, ph in enumerate('xyz'):
            amp = clip(Ir_xyz[i] / self.mit.Ns_Nr * 0.33, -1.0, 1.0)
            tp = fabs(amp)
            tn = fabs(-amp)
            clp = hex_bezier('#ff6600', '#aaffaa', '#0066ff', amp * 0.5 + 0.5)
            cln = hex_bezier('#ff6600', '#aaffaa', '#0066ff', -amp * 0.5 + 0.5)

            if amp < 0:
                clp, cln = cln, clp
                tp, tn = tn, tp

            self.prims['rotor']['coil']['in'][ph].fill = clp
            self.prims['rotor']['coil']['in'][ph].stroke = clp
            self.prims['rotor']['coil']['out'][ph].fill = cln
            self.prims['rotor']['coil']['out'][ph].stroke = cln

            self.prims['rotor']['current']['in'][ph].stroke = scale_rgb(clp, contrast_color_scale)
            self.prims['rotor']['current']['in'][ph].fill = scale_rgb(clp, contrast_color_scale)
            self.prims['rotor']['current']['out'][ph].stroke = scale_rgb(cln, contrast_color_scale)
            self.prims['rotor']['current']['out'][ph].fill = scale_rgb(cln, contrast_color_scale)

            for j, _ in enumerate(self.prims['rotor']['current']['in'][ph]):
                self.prims['rotor']['current']['in'][ph][j].scale(tp, center=self.prims['rotor']['current']['in'][ph][j].anchor)
                self.prims['rotor']['current']['out'][ph][j].scale(tn, center=self.prims['rotor']['current']['out'][ph][j].anchor)

            if amp < 0:
                self.prims['rotor']['current']['in'][ph].rotate(pi)
                self.prims['rotor']['current']['out'][ph].rotate(pi)
                self.prims['rotor']['coil']['in'][ph].rotate(pi)
                self.prims['rotor']['coil']['out'][ph].rotate(pi)



    def update_fields(self, sim):

        width_factor = 8
        scale_factor = 1.2

        for i, ph in enumerate('abc'):
            self.prims['stator']['field']['vec'][ph].scale(sim['I_abc'][i] * scale_factor)
            self.prims['stator']['field']['lines'][ph].width = abs(sim['I_abc'][i]) * width_factor
            if sim['I_abc'][i] < 0.0:
                self.prims['stator']['field']['lines'][ph].rotate(pi)

        # B_arm = self.prims['stator']['field']['vec']['a'][0].to_complex() + self.prims['stator']['field']['vec']['b'][0].to_complex() + self.prims['stator']['field']['vec']['c'][0].to_complex()
        beta = 2*pi/3
        self.B_arm = complex(0, 0)
        for k in range(3): self.B_arm += rect(sim['I_abc'][k], k*beta)
        self.B_field = rect(self.i_field, self.th_rot - self.th_ref)
        self.B_liq = rect(self.v_grid, self.th_grid - self.th_ref)

        self.prims['stator']['field']['vec']['s'].scale(abs(self.B_arm) * scale_factor)
        self.prims['stator']['field']['vec']['s'].rotate(phase(self.B_arm))
        self.prims['stator']['field']['lines']['s'].width = abs(self.B_arm) * width_factor
        self.prims['stator']['field']['lines']['s'].rotate(pi + phase(self.B_arm))

        self.prims['stator']['field']['vec']['l'][0].scale(abs(self.B_liq) * scale_factor)
        self.prims['stator']['field']['vec']['l'].rotate(phase(self.B_liq))
        self.prims['stator']['field']['lines']['l'].width = abs(self.B_liq) * width_factor
        self.prims['stator']['field']['lines']['l'].rotate(phase(self.B_liq))



        self.prims['rotor']['field']['vec']['r'].scale(abs(self.B_field) * scale_factor)
        # self.prims['rotor']['field']['vec']['r'].rotate(phase(self.B_field))
        self.prims['rotor']['field']['lines']['r'].width = abs(self.B_field) * width_factor
        # self.prims['rotor']['field']['lines']['r'].rotate(phase(self.B_field))



        for ph in 'abcsl':
            self.prims['stator']['field']['vec'][ph].visible = self.en_stator_field_vec and (ph in self.sel_stator_field.key)

        for ph in 'r':
            self.prims['rotor']['field']['vec'][ph].visible = self.en_rotor_field_vec and (ph in self.sel_rotor_field.key)

        for ph in 'abcsl':
            self.prims['stator']['field']['lines'][ph].visible = self.en_stator_field_lines and (ph in self.sel_stator_field.key)


        for ph in 'r':
            self.prims['rotor']['field']['lines'][ph].visible = self.en_rotor_field_lines and (ph in self.sel_rotor_field.key)



    def update_fig0(self, sim):
        ax, axt = self.widgets['figs'][0].axes
        freqs = self.sel_fig0.key == 'f_grt'
        if freqs:
            Y_show = (self.f_grid - self.f_ref, self.f_rot - self.f_ref, float('nan'))
            ax.set_ylim(self.sel_fig0.value[0], self.sel_fig0.value[1])
            axt.set_ylim(-20, 110)
            # ax.set_ylim(self.sel_fig0.value.y.min, self.sel_fig0.value.y.max)
            # axt.set_ylim(self.sel_fig0.value.twinx.y.min, self.sel_fig0.value.twinx.y.max)
            ax.set_ylabel('f_grid, f_rot [Hz]')
            axt.set_ylabel('delta [°]', color=cl['t'])
            axt.tick_params(axis='y', labelcolor=cl['t'])
            self.plt_lines['fig0_hline'].set_ydata((60.0,))
            self.plt_lines['fig0t_hline'].set_ydata((0.0,))
        else:
            Y_show = sim[self.sel_fig0.key]
            # if isinstance(self.sel_fig0.value, PlotLimits):
            #     ax.set_ylim(self.sel_fig0.value.y.min, self.sel_fig0.value.y.max)
            #     axt.set_ylim(self.sel_fig0.value.y.min, self.sel_fig0.value.y.max)
            # else:
            #     y_max = self.sel_fig0.value
            #     ax.set_ylim(-y_max, y_max)
            #     axt.set_ylim(-y_max, y_max)
            y_max = self.sel_fig0.value
            ax.set_ylim(-y_max, y_max)
            axt.set_ylim(-y_max, y_max)

            ax.set_ylabel(self.sel_fig0.key)
            axt.set_ylabel('', color='white')
            axt.tick_params(axis='y', labelcolor='white')
            self.plt_lines['fig0_hline'].set_ydata((np.nan,))
            self.plt_lines['fig0t_hline'].set_ydata((np.nan,))

        self.plt_t.append(self.t)
        self.plt_yy.append(0)
        for i, ph in enumerate(self.sel_fig0.key[2:5]):
            self.plt_y[i].append(Y_show[i])    #amp * sin(self.thg + phi + alpha * i))

            if freqs:
                yy = (self.delta)
                self.plt_yy[-1] = ( yy / pi * 180)
                self.plt_lines['twinx'].set_ydata(self.plt_yy)
                self.plt_lines['twinx'].set_xdata(self.plt_t)
                self.plt_lines['twinx_marker'].set_ydata((self.plt_yy[-1],))
                self.plt_lines['twinx_marker'].set_xdata((self.plt_t[-1],))
                self.plt_lines['twinx'].set_color(cl['t'])
                self.plt_lines['twinx_marker'].set_color(cl['t'])
            else:
                self.plt_lines['twinx'].set_ydata((np.nan,))
                self.plt_lines['twinx'].set_xdata((0,))
                self.plt_lines['twinx_marker'].set_ydata((np.nan,))
                self.plt_lines['twinx_marker'].set_xdata((0,))


            key = 'abc'[i]
            self.plt_lines[key].set_ydata(self.plt_y[i])
            self.plt_lines[key].set_xdata(self.plt_t)
            self.plt_lines[key + '_marker'].set_ydata((self.plt_y[i][-1],))
            self.plt_lines[key + '_marker'].set_xdata((self.plt_t[-1],))
            self.plt_lines[key].set_color(cl[ph])
            self.plt_lines[key + '_marker'].set_color(cl[ph])




        pad = 0.05 / self.time_factor * self.plot_downsample_factor
        t_max = max(self.plt_t)
        t_min = min(self.plt_t)
        # t_min = t_max -.01

        ax.set_xlim(t_min + pad, t_max + pad)



    def update_fig1(self, sim):
        ax = self.widgets['figs'][1].axes[0]

        delta = np.linspace(-pi*0.1, pi*1.1, 100)
        E, I, Te, Pe, Sout, V = self.gs.solve_Te_vs_delta(self.v_grid, delta, If=self.i_field, return_type=tuple)

        self.plt_lines['Tturb'].set_ydata((self.Tturb*2*pi*60.0,))
        self.plt_lines['delta'].set_xdata((phase(sim['E']) - phase(sim['V']),))
        self.plt_lines['Tind'].set_ydata(Sout.real)
        self.plt_lines['Tind'].set_xdata(delta)

        # self.plt_lines['Pturb_marker'].set_ydata((self.Tturb*2*pi*60.0,))
        # self.plt_lines['Pturb_marker'].set_xdata((phase(sim['E']) - phase(sim['V']),))

        self.plt_lines['Pout_marker'].set_ydata((sim['Sout'].real,))
        self.plt_lines['Pout_marker'].set_xdata((phase(sim['E']) - phase(sim['V']),))

        # self.plt_lines['Ix'].set_color(cl[self.sel_fig1.key])
        # self.plt_lines['Ix_marker'].set_color(cl[self.sel_fig1.key])
        # ax.set_xlim(min(nrs), max(nrs))
        ax.set_ylabel(self.sel_fig1.key + ((', ' + self.sel_fig1.key) if self.sel_fig1.key != 'nan' else ''))

    def update_fig2(self, sim):
        ax = self.widgets['figs'][1].axes[1]

        self.plt_lines['PQ_marker'].set_xdata((sim['Sout'].real * self.sel_Pout_unit.value,))
        self.plt_lines['PQ_marker'].set_ydata((sim['Sout'].imag * self.sel_Pout_unit.value,))



    def update_fps_info(self, dt: float):
        self.dt_filter_buffer.append(dt)
        fps = 1 / mean(self.dt_filter_buffer)
        self.widgets['fps'].config(text=f"{fps:.0f} fps")

    def destroy(self):
        del self.prims['stator']
        del self.prims['rotor']
        self.canvas.delete('all')


    def create_primis(self):
        gs_draw(self.canvas, self.prims, self.sel_stator_turns.value[0], self.sel_rotor_turns.value[0])
        self.prims.draw(consolidate_transforms_to_original=True)



    def update_info(self, sim: dict):

        self.widgets['w_stator_um'].config(text=f'[{self.sel_fs_unit.key}]')
        self.widgets['w_rotor_um'] .config(text=f'[{self.sel_fr_unit.key}]')
        self.widgets['w_grid_um']  .config(text=f'[{self.sel_fg_unit.key}]')
        self.widgets['Pconv_um']   .config(text=f'[{self.sel_Pout_unit.key}]')

        maxlen = len('stator [rad/s]')
        self.widgets['w_stator'].config(text=f'{self.f_ref * self.sel_fs_unit.value        :4.1f}')
        self.widgets['w_rotor'] .config(text=f'{self.f_rot * self.sel_fr_unit.value        :4.1f}')
        self.widgets['w_grid']  .config(text=f'{self.f_grid * self.sel_fg_unit.value       :4.1f}')
        self.widgets['Pconv']   .config(text=f"{sim['Sout'].real * self.sel_Pout_unit.value :4.2f}")
        self.widgets['delta']   .config(text=f'{self.delta / pi * 180 :4.2f}')
        # self.widgets['Tind']    .config(text=f'{sim['Te'] * self.sel_Te_unit.value         :4.2f}')
        self.widgets['Tind'].config(text=f'{self.v_grid * self.sel_Vg_unit.value         :4.2f}')
        self.widgets['f']       .config(text=f'{self.i_field             :4.2f}')
        self.widgets['time_factor'].config(text=f"{self.time_factor:>6.1f} x")

    def reset_time(self, reset_and_stop=False):
        if self.t is not None:
            self.invalidate_fig0_data()

        self._t_init = time.time()
        self.time_factor = 140
        self._t_start = 0.0
        self.th_rot = 0.0
        self.th_ref = 0.0
        self.th_grid = 0.0
        self.t = 0.0
        self.f_rot = 60.0
        self.f_ref = 0.0
        self.f_grid = 60.0
        self.B_liq = complex(0.0, 0.0)
        self.B_arm = complex(0.0, 0.0)
        self.B_field = complex(0.0, 0.0)
        self.v_grid = 1.0
        self.i_field = 1.0
        self._f_ref_inc = 0.0
        self._f_rot_inc = 0.0
        self._f_grid_inc = 0.0

        if reset_and_stop:
            self.run = False
            # self.draw_all()

    def invalidate_fig0_data(self, previous_time_factor=None):

        if not previous_time_factor:
            previous_time_factor = self.time_factor

        t_max = max(self.plt_t)
        for k in range(len(self.plt_t)):
            self.plt_t[k] = (self.plt_t[k] - t_max) * previous_time_factor / self.time_factor + t_max

        nans = np.empty(self.ax_npt_fig0, float)
        nans.fill(np.nan)
        for arr in (*self.plt_y, self.plt_yy):
            arr.extend(nans)

        self.plt_yy.extend(nans)


    @property
    def delta(self):
        delta = (self.th_rot - self.th_grid) % (2 * pi)
        if delta > pi:
            delta -= 2 * pi
        return delta


    def binds(self) -> None:

        def show_binds():
            current_state = self.run
            self.run = False
            messagebox.showinfo("binds", binds_message)
            self.run = current_state

        def toggle_run():
            if self.freeze:
                self.run = True
                self.freeze = False
            else:
                self.run = not self.run

        def toggle_sim():
            if self.en_sim_inertia:
                self.en_sim_inertia = False
            else:
                self.en_sim_inertia = True

        def inc_value(var_name: Literal['v', 'fg', 'fs', 'fr', 'delay', 'time_factor', 'Tres'],
                      increment: int | float,
                      v_min: int | float,
                      v_max: int | float):

            match var_name:
                case 'fs':
                        self._f_ref_inc = clip(self.f_ref + increment, v_min, v_max) - self.f_ref
                case 'fr':
                        self._f_rot_inc = clip(self.f_rot + increment, v_min, v_max) - self.f_rot
                case 'fg':
                        self._f_grid_inc = clip(self.f_grid + increment, v_min, v_max) - self.f_grid
                case 'Tturb':
                        self.Tturb = clip(self.Tturb + increment, v_min, v_max)
                case 'v':
                        self.v_grid = clip(self.v_grid + increment, v_min, v_max)
                case 'e':
                    self.i_field = clip(self.i_field + increment, v_min, v_max)
                case 'delay':
                    self.frame_delay = int(clip(self.frame_delay + increment, v_min, v_max))
                    print(f'(/*) time factor: {self.frame_delay}')
                case 'time_factor':
                    last = self.time_factor
                    self.time_factor = clip(self.time_factor + increment, v_min, v_max)
                    self.invalidate_fig0_data(last)

        def change_slots(parts: Literal['rotor', 'stator']):
            if isinstance(parts, str):
                parts = [parts]

            self.destroy()
            if 'rotor' in parts: next(self.sel_rotor_turns)
            if 'stator' in parts: next(self.sel_stator_turns)
            self.create_primis()


        def reset_colors():
            next(self.sel_dynamic_color)
            if not self.sel_dynamic_color.value:
                self.widgets['stator_coil_front'].config(state=tk.NORMAL)
                self.widgets['rotor_coil_front'].config(state=tk.NORMAL)
                # del self.en_stator_coil_front
                # del self.en_rotor_coil_front
                self.en_stator_coil_front = BoolVar(False)
                self.en_rotor_coil_front = BoolVar(False)
                self.widgets['stator_coil_front'].configure(variable=self.en_stator_coil_front)
                self.widgets['rotor_coil_front'].configure(variable=self.en_rotor_coil_front)
                for ph in 'abc':
                    for direction in ('in', 'out'):
                        self.prims['stator']['coil'][direction][ph].stroke = scale_rgb(cl[ph], contrast_color_scale)
                        self.prims['stator']['coil'][direction][ph].fill = cl[ph]
                    for direction in ('in', 'out'):
                        self.prims['stator']['current'][direction][ph].fill = scale_rgb(cl[ph], contrast_color_scale)
                        self.prims['stator']['current'][direction][ph].stroke = scale_rgb(cl[ph], contrast_color_scale)

                    for ph in 'xyz':
                        for direction in ('in', 'out'):
                            self.prims['rotor']['coil'][direction][ph].stroke = scale_rgb(cl[ph], contrast_color_scale)
                            self.prims['rotor']['coil'][direction][ph].fill = cl[ph]
                        for direction in ('in', 'out'):
                            self.prims['rotor']['current'][direction][ph].fill = scale_rgb(cl[ph], contrast_color_scale)
                            self.prims['rotor']['current'][direction][ph].stroke = scale_rgb(cl[ph], contrast_color_scale)
            else:
                self.en_stator_coil_front = False
                self.en_rotor_coil_front = False
                self.widgets['stator_coil_front'].config(state=tk.DISABLED)
                self.widgets['rotor_coil_front'].config(state=tk.DISABLED)


        def change_coil_opacity():
            next(self.sel_coil_opacity)
            for ph in ('r'):
                for node in self.prims['rotor'].filter_matching_keys('coil_front', ph):
                    node.fill = cl[ph]
                    node.stipple = self.sel_coil_opacity.key
            for ph in ('abc'):
                for node in self.prims['stator'].filter_matching_keys('coil_front', ph):
                    node.fill = cl[ph]
                    node.stipple = self.sel_coil_opacity.key

        def change_part_visibility():
            x, y = self.mouse_norm_coords
            if collision_circle_point(self.prims['rotor']['core']['outer'][0], (x, y)):
                self.prims['rotor'].toggle_visible()
            elif collision_circle_point(self.prims['stator']['core']['outer'][0], (x, y)):
                self.prims['stator'].toggle_visible()
            else:
                return

        def toggle_autoscale(selection, axis):
            if axis in ('x', 'both'):
                selection.value.x.autoscale = not selection.value.x.autoscale
            if axis in ('y', 'both'):
                selection.value.y.autoscale = not selection.value.y.autoscale
            print(selection.value.y.autoscale)


        def field_menu(event, selection):
            self.run = False
            self._previous_run = False
            menu = tk.Menu(self.canvas.window, tearoff=0)
            if selection.name is not None:
                menu.add_command(label=selection.name, state='disabled')
                menu.add_separator()
            for it in selection:
                menu.add_command(label=it, command=partial(selection.set_current_key, it))
            # if isinstance(selection.value, PlotLimits):
            #     menu.add_separator()
            #     menu.add_command(label='y autoscale', command=partial(toggle_autoscale, selection, 'y'))
            menu.tk_popup(event.x_root, event.y_root)
            self.run = True










        def toggle_freeze():
            self.freeze = not self.freeze
            if not self.freeze:
                self.run = True

        def reload():
            global __reload
            globals.set_reload(True)
            self._close_next_refresh = True


        dw_inc = 0.89   #0.83333333333333333333333333
        f_max = 70
        self.canvas.window.bind('<Prior>', lambda event: inc_value('fs', 0.25, -f_max, f_max))
        self.canvas.window.bind('<Next>', lambda event: inc_value('fs', -0.25, -f_max, f_max))

        self.canvas.window.bind(',', lambda event: inc_value('fr', -0.25, -f_max, f_max))
        self.canvas.window.bind('.', lambda event: inc_value('fr', 0.25, -f_max, f_max))
        # self.canvas.window.bind('<', lambda event: inc_value('fr', -10*0.25, -f_max, f_max))
        # self.canvas.window.bind('>', lambda event: inc_value('fr', 10*0.25, -f_max, f_max))

        self.canvas.window.bind('<Right>', lambda event: inc_value('fg', 0.25, -f_max, f_max))
        self.canvas.window.bind('<Left>', lambda event: inc_value('fg', -0.25, -f_max, f_max))
        self.canvas.window.bind('<Shift-Right>', lambda event: inc_value('fg', 10*0.25, -f_max, f_max))
        self.canvas.window.bind('<Shift-Left>', lambda event: inc_value('fg', -10*0.25, -f_max, f_max))

        self.canvas.window.bind('0', lambda event: inc_value('e', increment= 1/16, v_min=1/4, v_max=7/4))
        self.canvas.window.bind('9', lambda event: inc_value('e', increment=-1/16, v_min=1/4, v_max=7/4))
        self.canvas.window.bind(')', lambda event: inc_value('e', increment=1 / 4, v_min=1 / 4, v_max=7 / 4))
        self.canvas.window.bind('(', lambda event: inc_value('e', increment=-1 / 4, v_min=1 / 4, v_max=7 / 4))
        self.canvas.window.bind('=', lambda event: inc_value('v', increment= 1/16, v_min=1/4, v_max=7/4))
        self.canvas.window.bind('-', lambda event: inc_value('v', increment=-1/16, v_min=1/4, v_max=7/4))
        self.canvas.window.bind('+', lambda event: inc_value('v', increment=1/4, v_min=1/4, v_max=7/4))
        self.canvas.window.bind('_', lambda event: inc_value('v', increment=-1/4, v_min=1/4, v_max=7/4))

        self.canvas.window.bind('<Up>', lambda event: inc_value('Tturb', self.Tturb0/50, -self.Tturb0, self.Tturb0))
        self.canvas.window.bind('<Down>', lambda event: inc_value('Tturb', -self.Tturb0/50, -self.Tturb0, self.Tturb0))
        self.canvas.window.bind('<Shift-Up>', lambda event: inc_value('Tturb', 10*self.Tturb0/50, -self.Tturb0, self.Tturb0))
        self.canvas.window.bind('<Shift-Down>', lambda event: inc_value('Tturb', -10*self.Tturb0/50, -self.Tturb0, self.Tturb0))



        self.canvas.window.bind(']', lambda event: inc_value('time_factor', self.time_factor*.2, 32.45273575943723503208293147346, 1492.992))
        self.canvas.window.bind('[', lambda event: inc_value('time_factor', -self.time_factor*.16666666666666666666666666666667, 32.45273575943723503208293147346, 1492.992))

        # self.canvas.window.bind('v', lambda event: next(self.select_speed_unit) )
        # self.canvas.window.bind('p', lambda event: next(self.select_power_unit))
        self.canvas.window.bind('*',       lambda event: inc_value('delay', 1, 0, 30))
        self.canvas.window.bind('/',       lambda event: inc_value('delay', -1, 0, 30))
        self.canvas.window.bind('<space>', lambda event: toggle_run())
        # self.canvas.window.bind('<Return>', lambda event: toggle_sim())
        self.canvas.window.bind('<F1>', lambda event: show_binds())
        self.canvas.window.bind('<F4>', lambda event: toggle_freeze())
        self.canvas.window.bind('<F5>', lambda event: reload())
        self.canvas.window.bind('<Escape>', lambda event: self.reset_time(reset_and_stop=True))
        self.canvas.window.bind('<End>',     lambda event: self.reset_time())

        self.canvas.window.bind('m', lambda event: change_slots('stator'))
        self.canvas.window.bind('n', lambda event: change_slots('rotor'))
        self.canvas.window.bind('o', lambda event: change_coil_opacity())

        # self.canvas.window.bind('d', lambda event: self.destroy())
        # self.canvas.window.bind('c', lambda event: self.create())
        self.widgets['canvas_fig0'].get_tk_widget().bind('<Button-1>', lambda event: {next(self.sel_fig0), self.invalidate_fig0_data()})
        self.widgets['canvas_fig0'].get_tk_widget().bind('<Button-3>', lambda event: {self.invalidate_fig0_data(), field_menu(event, self.sel_fig0)})
        self.widgets['sim_inertia'].bind('<Button-3>', lambda event: {field_menu(event, self.dumping)})

        self.widgets['canvas_fig1'].get_tk_widget().bind('<Button-1>', lambda event: next(self.sel_fig1))
        self.widgets['canvas_fig1'].get_tk_widget().bind('<Button-3>', lambda event: {field_menu(event, self.dumping)})
        # self.canvas.bind('<Button-3>', lambda event: {next(self.select_part),self.prims.print_tree(print_leafs=False)})
        self.canvas.bind('<Button-1>', lambda event: change_part_visibility())
        self.canvas.bind('<Button-3>', lambda event: field_menu(event, self.sel_stator_field))

        self.canvas.window.bind('T',  lambda event: {print(f"\n\n{'-' * 135}"), self.prims.print_tree()})
        self.canvas.window.bind('t',  lambda event: {print(f"\n\n{'-'*135}"), self.prims.print_tree(False)})
        self.canvas.window.bind('d',  lambda event: reset_colors())
        self.canvas.window.bind('s', lambda event: next(self.sel_stator_field))
        self.canvas.window.bind('r', lambda event: next(self.sel_rotor_field))
        self.canvas.window.bind('\\', lambda event: next(self.sel_mount))   # todo: implementar colisão com polígono e usar mouse
        # self.canvas.window.bind('k',  lambda event: next(self.select_ref))


        self.widgets['sim_inertia']       .configure(variable=self.en_sim_inertia)
        self.widgets['stator_field_lines'].configure(variable=self.en_stator_field_lines)
        self.widgets['stator_field_vec']  .configure(variable=self.en_stator_field_vec)
        self.widgets['rotor_field_lines'] .configure(variable=self.en_rotor_field_lines)
        self.widgets['rotor_field_vec']   .configure(variable=self.en_rotor_field_vec)
        self.widgets['stator_coil_front'] .configure(variable=self.en_stator_coil_front)
        self.widgets['rotor_coil_front']  .configure(variable=self.en_rotor_coil_front)

        self.widgets['w_stator_um'].bind('<Button-1>', lambda event: next(self.sel_fs_unit))
        self.widgets['w_rotor_um'] .bind('<Button-1>', lambda event: next(self.sel_fr_unit))
        self.widgets['w_grid_um']  .bind('<Button-1>', lambda event: next(self.sel_fg_unit))
        self.widgets['Pconv_um']   .bind('<Button-1>', lambda event: next(self.sel_Pout_unit))

        self.widgets['ref_free'].configure(variable=self.sel_reference_frame, value='free')
        self.widgets['ref_stator'].configure(variable=self.sel_reference_frame, value='stator')
        self.widgets['ref_rotor'] .configure(variable=self.sel_reference_frame, value='rotor')
        self.widgets['ref_field'] .configure(variable=self.sel_reference_frame, value='field')

        # self.canvas.window.bind('<B1-Motion>', lambda event: drag_window())

        # todo: lista em F1