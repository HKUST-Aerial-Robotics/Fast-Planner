import numpy as np
import sympy as sp


def calcInv():
    dt = sp.symbols('dt')

    A = sp.Matrix([[1 / 6.0, 2 / 3.0, 1 / 6.0],
                   [-1 / (2.0 * dt), 0, 1 / (2.0 * dt)],
                   [1 / (dt * dt), -2 / (dt * dt), 1 / (dt * dt)]])

    print A.inv()
    sp.pprint(A.inv())


def calcObj():
    # p1 = sp.symbols('a1')
    # p2 = sp.symbols('a2')
    # p3 = sp.symbols('a3')
    # p4 = sp.symbols('a4')

    # c1 = sp.symbols('c1')
    # c2 = sp.symbols('c2')

    # f = (p4 - 3 * p3 + 3 * p2 - p1)**2 + (
    #     p1 / 6.0 + 2 * p2 / 3.0 + p3 / 6.0 - c1)**2 + (
    #         p2 / 6.0 + 2 * p3 / 3.0 + p4 / 6.0 - c2)**2

    # f = sp.expand(f)
    # f = sp.simplify(f)

    # sp.pprint(f)

    p = sp.symbols('p:9')
    w = sp.symbols('w:7')

    cost = 0

    for i in range(0, 6):
        cost += (p[i + 3] - 3 * p[i + 2] + 3 * p[i + 1] - p[i])**2

    print cost

    for i in range(0, 7):
        cost += (p[i] / 6 + 2 * p[i + 1] / 3 + p[i + 2] / 6 - w[i])**2

    print(sp.expand(cost))
    # sp.pprint(sp.expand(cost))


if __name__ == "__main__":
    # calcInv()
    calcObj()
