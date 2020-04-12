import numpy as np
import sympy as sp
from calc_jacobian import calcJacobian


def distToLine():
    """ compute the distance of a point qb to a line qiqj, and find its gradient wrt qi, qj """
    # define vars
    qix, qiy, qiz = sp.symbols('qix, qiy, qiz')
    qjx, qjy, qjz = sp.symbols('qjx, qjy, qjz')
    qbx, qby, qbz = sp.symbols('qbx, qby, qbz')
    qi = sp.Matrix([qix, qiy, qiz])
    qj = sp.Matrix([qjx, qjy, qjz])
    qb = sp.Matrix([qbx, qby, qbz])

    # write cost function
    u = (qb - qi).cross(qj - qi)
    fu = u.dot(u)
    v = (qj - qi)
    fv = v.dot(v)
    f = fu / fv
    f = sp.simplify(f)
    print(f)

    # find jacobian
    df_dqi = calcJacobian([f], [qix, qiy, qiz]).transpose()
    df_dqi = sp.simplify(df_dqi)
    print df_dqi
    df_dqj = calcJacobian([f], [qjx, qjy, qjz]).transpose()
    df_dqj = sp.simplify(df_dqj)


def distToLine2():
    """ compute the distance of a point qb to a line qiqj, and find its gradient wrt qi, qj """
    # define vars
    qix, qiy, qiz = sp.symbols('qix, qiy, qiz')
    qjx, qjy, qjz = sp.symbols('qjx, qjy, qjz')
    qbx, qby, qbz = sp.symbols('qbx, qby, qbz')
    qi = sp.Matrix([qix, qiy, qiz])
    qj = sp.Matrix([qjx, qjy, qjz])
    qb = sp.Matrix([qbx, qby, qbz])

    # write cost function
    u = (qb - qi).cross(qj - qi)
    print sp.simplify(u)
    return

    fu = u.dot(u)
    v = (qj - qi)
    fv = v.dot(v)
    f = fu / fv
    f = sp.simplify(f)
    print(f)

    # find jacobian
    df_dqi = calcJacobian([f], [qix, qiy, qiz]).transpose()
    df_dqi = sp.simplify(df_dqi)
    print df_dqi
    df_dqj = calcJacobian([f], [qjx, qjy, qjz]).transpose()
    df_dqj = sp.simplify(df_dqj)


def distToView():
    px, py, pz = sp.symbols('px, py, pz')
    vx, vy, vz = sp.symbols('vx, vy, vz')
    qix, qiy, qiz = sp.symbols('qix, qiy, qiz')
    p = sp.Matrix([px, py, pz])
    v = sp.Matrix([vx, vy, vz])
    qi = sp.Matrix([qix, qiy, qiz])

    # dist to view
    d = (qi - p) - ((qi - p).dot(v)) * v
    D = d.dot(d)
    dD_dqi = calcJacobian([D], [qix, qiy, qiz]).transpose()
    dD_dqi = sp.simplify(dD_dqi)
    sp.pprint(dD_dqi)
    print dD_dqi

    dd_dqi = calcJacobian([d[0], d[1], d[2]], [qix, qiy, qiz])
    dd_dqi = sp.simplify(dd_dqi)
    print dd_dqi

if __name__ == "__main__":
    # distToLine2()
    distToView()
