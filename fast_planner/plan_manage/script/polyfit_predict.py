import numpy as np
import sympy as sp

def polyfit():
    p0 = sp.symbols('p0')
    p1 = sp.symbols('p1')
    p2 = sp.symbols('p2')
    p3 = sp.symbols('p3')
    p4 = sp.symbols('p4')
    p5 = sp.symbols('p5')
    t = sp.symbols('t')

    # ti = np.array(0., 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0])

    # poly = p0 + p1 * t + p2 * t**2 + p3 * t**3 + p4 * t**4 + p5 * t**5
    # print poly

    # print ti[0]

    # poly = p0 + p1 * ti[2] + p2 * ti[2]**2 + p3 * ti[2]**3 + p4 * ti[
    #     2]**4 + p5 * ti[2]**5
    # print poly

    # calc the estimation error terms
    ti = sp.symbols('ti')
    qi = sp.symbols('qi')

    est_err = (p0 + p1 * ti + p2 * ti**2 + p3 * ti**3 + p4 * ti**4 + p5 * ti**5 - qi)**2
    est_err = sp.expand(est_err)
    print "est err:"
    print est_err
    print 

    # calc the regulator
    pos = p0 + p1 * t + p2 * t**2 + p3 * t**3 + p4 * t**4 + p5 * t**5
    acc = sp.diff(pos, t, 2)
    acc =sp.simplify(acc)

    acc2 = acc *acc
    # reg = sp.integrate(sp.sin(t), (t, 0, 3.141))
    t1 = sp.symbols('t1')
    t2 = sp.symbols('t2')
    reg = sp.integrate(acc2, (t, t1, t2))
    reg = sp.simplify(reg)
    print 'reg:'
    print reg
    print

    # sum of cost
    ld = sp.symbols('ld')
    
    sum = est_err + ld * reg
    print 'sum:'
    print sum


    print 'grad:'
    grad = sp.simplify(sp.diff(sum, p0))
    print grad
    print
    grad = sp.simplify(sp.diff(sum, p1))
    print grad
    print
    grad = sp.simplify(sp.diff(sum, p2))
    print grad
    print
    grad = sp.simplify(sp.diff(sum, p3))
    print grad
    print
    grad = sp.simplify(sp.diff(sum, p4))
    print grad
    print
    grad = sp.simplify(sp.diff(sum, p5))
    print grad
    print



if __name__ == "__main__":
    polyfit()
