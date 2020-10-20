import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from pyefd import elliptic_fourier_descriptors


im = cv.imread(cv.samples.findFile("images/black_ellipse.jpg"))
gray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
ret, binary = cv.threshold(gray, 215, 255, cv.THRESH_BINARY)
contours, hierarchy = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

order = 4

coeffs = []
for cnt in contours:
    # Find the coefficients of all contours
    coeffs.append(elliptic_fourier_descriptors(np.squeeze(cnt), order=4))

dxy = np.diff(np.squeeze(contours[1]), axis=0)
dt = np.sqrt((dxy ** 2).sum(axis=1))
t = np.concatenate([([0.]), np.cumsum(dt)])
T = t[-1]

phi = (2 * np.pi * t) / T

orders = np.arange(1, order + 1)
const = (2 * orders * np.pi) / T

hmn = const.reshape((order, -1))*t

a = coeffs[1][:, 0] * const
b = coeffs[1][:, 1] * const
c = coeffs[1][:, 2] * const
d = coeffs[1][:, 3] * const

a = a.reshape((order, -1))
b = b.reshape((order, -1))
c = c.reshape((order, -1))
d = d.reshape((order, -1))

Tx = -1*a*np.sin(hmn) + b*np.cos(hmn)
Ty = -1*c*np.sin(hmn) + d*np.cos(hmn)

Nx = -1*a*hmn*hmn*np.cos(hmn) - b*hmn*hmn*np.sin(hmn)
Ny = -1*c*hmn*hmn*np.cos(hmn) - d*hmn*hmn*np.sin(hmn)

print(np.shape(Tx))
print(np.shape(Ty))
print(np.shape(Nx))
print(np.shape(Ny))
