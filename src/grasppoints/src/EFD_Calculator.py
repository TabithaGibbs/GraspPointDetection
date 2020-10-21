import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from pyefd import elliptic_fourier_descriptors




class EFD_Calculator:

    def __init__(self, order):
        self.order = order

    def calc_coeffs(self, max_contour):
        coeffs = []
        coeffs.append(elliptic_fourier_descriptors(np.squeeze(max_contour), order=self.order))

        dxy = np.diff(np.squeeze(max_contour), axis=0)
        dt = np.sqrt((dxy ** 2).sum(axis=1))
        t = np.concatenate([([0.]), np.cumsum(dt)])
        T = t[-1]

        phi = (2 * np.pi * t) / T

        orders = np.arange(1, self.order + 1)
        const = (2 * orders * np.pi) / T

        hmn = const.reshape((self.order, -1)) * t

        a = coeffs[0][:, 0] * const
        b = coeffs[0][:, 1] * const
        c = coeffs[0][:, 2] * const
        d = coeffs[0][:, 3] * const

        a = a.reshape((self.order, -1))
        b = b.reshape((self.order, -1))
        c = c.reshape((self.order, -1))
        d = d.reshape((self.order, -1))

        Tx = -1 * a * np.sin(hmn) + b * np.cos(hmn)
        Ty = -1 * c * np.sin(hmn) + d * np.cos(hmn)

        Nx = -1 * a * hmn * hmn * np.cos(hmn) - b * hmn * hmn * np.sin(hmn)
        Ny = -1 * c * hmn * hmn * np.cos(hmn) - d * hmn * hmn * np.sin(hmn)

        Tx = np.sum(Tx,0)
        Ty = np.sum(Ty,0)
        Nx = np.sum(Nx,0)
        Ny = np.sum(Ny,0)



        return [Tx, Ty, Nx, Ny]
