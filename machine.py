from math import pi
from cmath import polar, rect
import numpy as np


class SynchronousGenerator:
    def __init__(self, E_nom:float, f_nom:float, Xs:float, R:float, p:int=2):
        self.Xs = Xs
        self.R = R
        self.E_nom = E_nom
        self.f_nom = f_nom
        self.p = p

    def solve(self, V: complex, If: float = 1.0, theta_E: float = 0.0, f: float = None, return_type=tuple):
        if f is None:
            f = self.f_nom
        Xs = self.Xs if f is None else self.Xs * f / self.f_nom
        E = rect(self.E_nom / self.f_nom * f, theta_E) * If
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


    def solve_Te_vs_delta(self, V_bar: float, delta: np.ndarray, If=1.0, f=None, return_type=dict):

        V = np.empty_like(delta, dtype=complex)
        E = np.empty_like(delta, dtype=complex)
        Te = np.empty_like(delta, dtype=float)
        Pe = np.empty_like(delta, dtype=float)
        I = np.empty_like(delta, dtype=complex)
        Sout = np.empty_like(delta, dtype=complex)
        for k in range(len(delta)):
            E[k], I[k], Te[k], Pe[k], Sout[k], V[k] = self.solve(V_bar, If=If, theta_E=delta[k], f=f, return_type=tuple)

        if return_type == tuple:
            return E, I, Te, Pe, Sout, V
        if return_type == dict:
            return {'E': E, 'I': I, 'Te': Te, 'Pe': Pe, 'Sout': Sout, 'V': V}
        raise ValueError("'return_type' de ve ser 'tuple' ou 'dict")



class InductionMotor:
    def __init__(self, V1nom: float, fnom: float, poles: float, R1: float, R2: float, Rf: float, X1: float, X2: float, Xm: float):
        self.V1nom = V1nom
        self.fnom = fnom
        self.poles = poles
        self.R1 = R1
        self.R2 = R2
        self.Rf = Rf
        self.X1 = X1
        self.X2 = X2
        self.Xm = Xm
        self._Rinf = 10e6    # resistência de circuito aberto

    def n_from_f(self, f): return 120.*f/ self.poles
    def w_from_f(self, f): return 4.0*pi*f/self.poles
    def w_from_n(self, n): return n*pi/30.0

    def solve(self, nr: float, V1: float = None, f: float = None):
        if f is None:
            f = self.fnom

        if V1 is None:
            V1 = self.V1nom

        ns = self.n_from_f(f)
        if ns != 0.0:
            s = (ns - nr) / ns
            Rconv = (1.0 - s) / s
        else:
            s = 0.0
            Rconv = self._Rinf

        X1 = self.X1 * f / self.fnom
        X2 = self.X2 * f / self.fnom
        Xm = self.Xm * f / self.fnom

        Z0 = self.Rf * complex(0.0, Xm) / (self.Rf + complex(0.0, Xm))
        Z1 = complex(self.R1, X1)
        Z2 = complex(self.R2 + Rconv, X2)

        Z02 = Z0 * Z2 / (Z0 + Z2)
        Zeq = Z1 + Z02

        I1 = V1 / Zeq
        E = I1 * Z02
        I2 = E / Z2
        Im = E / complex(0.0, Xm)

        Sin = 3 * V1 * I1.conjugate()
        Pin = abs(Sin)
        Pef = abs(3 * E * I2.conjugate())
        Pconv = abs(3 * abs(I2)**2 * Rconv)

        Tind = Pef / self.w_from_f(f) if f != 0.0 else 0.0
        return {'I1': I1,
                'I2': I2,
                'Im': Im,
                'V1': V1,
                'f': f,
                'ns': ns,
                'nr': nr,
                'E': E,
                's': s,
                'Pconv': Pconv,
                'Pin': Pin,
                'Sin': Sin
        }





