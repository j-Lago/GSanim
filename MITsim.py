from cmath import polar, rect, phase
from math import pi
import numpy as np
import warnings

class MITsim:
    def __init__(self, R1: float, X1: float, R2: float, X2: float, Rc: float, Xm: float, p: int = 2,
                 V1nom: float = 1.0, fnom: float = 60.0, fcomp0: float = 10.0, mcomp0: float = 0.017, Ns_Nr=150):

        self.mcomp0 = mcomp0
        self.fcomp0 = fcomp0

        self.p = p
        self.fnom = fnom
        self.V1nom = V1nom
        self.R1 = R1
        self.X1 = X1
        self.R2 = R2
        self.X2 = X2
        self.Rc = Rc
        self.Xm = Xm
        self.V1 = complex(V1nom + 0.0)
        self.wr = 0.0
        self.Ns_Nr = Ns_Nr

        self.k0 = 1.0
        self.k1 = -0.00
        self.k2 = 0.00001

        self.f = self.fnom
        self.I1 = complex(0.0 + 0.0) * 3
        self.E  = complex(0.0 + 0.0) * 3
        # self.Ic = complex(0.0 + 0.0) * 3
        self.Im = complex(0.0 + 0.0) * 3
        self.I2 = complex(0.0 + 0.0) * 3
        self.Pconv = 0.0
        self.Pef = 0.0
        self.Tind = 0.0
        self.Tres = 0.0
        self.nan = float('nan')   # hack para CircularDict -> mit['nan'] -> float('nan')

    @property
    def s(self):
        self.ws = (2.0 * pi * self.f) * (2.0 / self.p)
        return (self.ws - self.wr) / self.ws if self.ws != 0 else float('nan')

    @property
    def ns(self):
        return self.f * 2*60 / self.p

    @property
    def nr(self):
        return self.wr * 30.0 / pi

    def solve(self):

        s = self.s


        X1 = self.X1 * self.f / self.fnom
        X2 = self.X2 * self.f / self.fnom
        Xm = self.Xm * self.f / self.fnom

        Rconv = self.R2 * (1.0 - s) / s if s != 0.0 else 1e18

        Z0 = self.Rc * complex(0, Xm) / (self.Rc + complex(0, Xm))
        Z1 = self.R1 + complex(0, X1)
        Z2 = self.R2 + complex(0, X2) + Rconv

        Z02 = Z0*Z2 / (Z0+Z2)
        Zeq = Z02 + Z1

        try:
            self.I1 = self.V1 / Zeq
        except ZeroDivisionError:
            self.I1 = float('nan')

        self.E = self.I1 * Z02
        # self.Ic = self.E / self.Rc
        last_Im = self.Im
        try:
            self.Im = self.E / complex(0, Xm)
        except ZeroDivisionError:
            self.Im = last_Im


        self.I2 = self.E / Z2

        self.Pef = 3 * (self.E * self.I2.conjugate()).real
        self.Pconv = 3 * abs(self.I2)**2*Rconv


        try:
            self.Tind = self.Pef / self.ws
        except ZeroDivisionError:
            self.Tind = 0.0

        awr = abs(self.wr)
        self.Tres = awr**2 * self.k2 + awr * self.k1 + self.k0
        if self.wr < 0:
            self.Tres = -self.Tres

        dead_zone = 2
        if -dead_zone < self.wr < dead_zone:
            self.Tres = 0.0



    def solve_range(self, wr_start: float, wr_stop: float, samples: int, y_keys: list):

        wrs = np.linspace(wr_start, wr_stop, samples, dtype=float)




        data = {}
        for key in y_keys:
            data[key] = np.empty_like(wrs, dtype=type(self[key]))
        data['wr'] = wrs
        data['nr'] = wrs * 30.0 / pi

        for i, wr in enumerate(wrs):
            self.wr = wr
            self.solve()
            for key in y_keys:
                data[key][i] = self[key]

        return data



    def __getitem__(self, key):

        if key == 'nan':
            return self.wr * np.nan

        if key in self.__dict__:
            return self.__dict__[key]
        else:
            raise ValueError(f"MITsim não possui o atributo '{key}'.")


    def m_comp(self, compensate_Z1=True, clip=True):
        """
        Modulation index compensado pela frequência com saturação e step inicial
        """

        m = abs(self.f) / self.fnom

        if compensate_Z1:
            mcomp = self.mcomp0 + (self.fcomp0 / self.fnom - self.mcomp0) / self.fcomp0 * abs(self.f)
            m = max(m, mcomp)

        if clip:
            m = min(m, 1)

        return m




