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


if __name__ == "__main__":
    distToLine2()

