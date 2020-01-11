import numpy as np
import sympy as sp


def traj_opt():
    print

    # ---------------------- variable ----------------------

    # control points
    q = sp.symbols('q0:10')
    print q
    # print q[10]

    # distance
    dt = sp.symbols('dt')
    d = sp.symbols('d:10')
    print d
    # print d[10]

    # dynamic
    vm = sp.symbols('vm')
    am = sp.symbols('am')
    ts = sp.symbols('ts')

    lambda1 = sp.symbols('lambda1')
    lambda2 = sp.symbols('lambda2')
    lambda3 = sp.symbols('lambda3')
    lambda4 = sp.symbols('lambda4')

    sp.pprint(lambda1)
    sp.pprint(lambda3)

    # ---------------------- objective ----------------------

    # smoothness, jerk
    print '------------------------smoothness------------------------'
    Js = 0

    # for n in range(len(q)):
    for i in range(10):
        if i < 10 - 4:
            Js += (q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i])**2

    Js = sp.simplify(Js)
    sp.pprint(Js)
    print
    print Js

    # collision
    print '------------------------collision------------------------'
    Jc = 0

    # for n in range(len(q)):
    for i in range(10):
        Jc += (d[i] - dt)**2

    Jc = sp.simplify(Jc)
    sp.pprint(Jc)
    print
    print Jc

    # feasibility
    print '------------------------feasibility-----------------------'

    Jv = 0

    # velocity
    for i in range(10):
        if i < 10 - 1:
            Jv += 1


            


if __name__ == "__main__":
    traj_opt()