import numpy as np
import sympy as sp


def calcJacobian(funcs, vars):
    jacobi = []
    for func in funcs:
        j_row = []
        for var in vars:
            dif = sp.simplify(sp.diff(func, var))
            j_row.append(dif)
        jacobi.append(j_row)
    return sp.Matrix(jacobi)