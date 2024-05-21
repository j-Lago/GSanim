from math import pi
from cmath import polar, rect
import numpy as np


class GSsim:
    def __init__(self, E_nom:float, f_nom:float, Xs:float, R:float, p:int=2):
        self.Xs = Xs
        self.R = R
        self.E_nom = E_nom
        self.f_nom = f_nom
        self.p = p

    def solve(self, V: complex, theta_E: float = 0.0, f: float = None, return_type=tuple):
        if f is None:
            f = self.f_nom
        Xs = self.Xs if f is None else self.Xs * f / self.f_nom
        E = rect(self.E_nom / self.f_nom * f, theta_E)
        I = (E - V) / complex(self.R, Xs)
        Pe = 3 * (E * I.conjugate()).real
        w = f*pi*4.0/self.p
        Te = Pe / w
        Sout = 3 * V * I.conjugate()
        if return_type == tuple:
            return E, I, Te, Pe, Sout, V
        if return_type == dict:
            return {'E': E, 'I': I, 'Te': Te, 'Pe': Pe, 'Sout': Sout, 'V': V}
        raise ValueError("'return_type' de ve ser 'tuple' ou 'dict")


    def solve_Te_vs_delta(self, V_bar: float, delta: np.ndarray, f=None, return_type=tuple):

        V = np.empty_like(delta, dtype=complex)
        E = np.empty_like(delta, dtype=complex)
        Te = np.empty_like(delta, dtype=float)
        Pe = np.empty_like(delta, dtype=float)
        I = np.empty_like(delta, dtype=complex)
        Sout = np.empty_like(delta, dtype=complex)
        for k in range(len(delta)):
            E[k], I[k], Te[k], Pe[k], Sout[k], V[k] = self.solve(V_bar, delta[k], f)

        if return_type == tuple:
            return E, I, Te, Pe, Sout, V
        if return_type == dict:
            return {'E': E, 'I': I, 'Te': Te, 'Pe': Pe, 'Sout': Sout, 'V': V}
        raise ValueError("'return_type' de ve ser 'tuple' ou 'dict")





