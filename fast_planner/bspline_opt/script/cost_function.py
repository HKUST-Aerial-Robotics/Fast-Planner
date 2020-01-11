import numpy as np
import sympy as sp
from math import *
import matplotlib.pyplot as plt


def f1(x):
    return (np.sqrt(x) - 1.0)**2

def df1dx(x):
  return 


def drawCostFunction():
    x1 = np.arange(0.0, 5.0, 0.01)
    y1 = f1(x1)
    print x1
    print y1
    plt.figure()
    plt.plot(x1, y1)
    plt.show()


if __name__ == "__main__":
    drawCostFunction()