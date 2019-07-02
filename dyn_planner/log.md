
# Environment Rrepresentation
<!-- - dynamic obstacle predictor: use previous observation to predict a short range trajectory (distribution).  -->
<!-- - also implement sdf in a more compact way. -->
<!-- - Env: SDFMap + ObjPrediction, evaluate EDT -->
<!-- - refine evaluateEDT -->
<!-- - kinodynamic astar using Env -->
<!-- - hash table have shits. -->
- bspline optimization using. Env

- dealing with uncertainty. -wenchao paper

- refactor pcd path finder

- pnp


ld*(-4*p2**2*t1 + 4*p2**2*t2 - 12*p2*p3*t1**2 + 12*p2*p3*t2**2 - 80*p4*p5*t1**6 + 80*p4*p5*t2**6 - 400*p5**2*t1**7/7 + 400*p5**2*t2**7/7 - t1**5*(48*p3*p5 + 144*p4**2/5) - t1**4*(20*p2*p5 + 36*p3*p4) - t1**3*(16*p2*p4 + 12*p3**2) + t2**5*(48*p3*p5 + 144*p4**2/5) + t2**4*(20*p2*p5 + 36*p3*p4) + t2**3*(16*p2*p4 + 12*p3**2)) + p0**2 + 2*p0*p1*ti + 2*p0*p2*ti**2 + 2*p0*p3*ti**3 + 2*p0*p4*ti**4 + 2*p0*p5*ti**5 - 2*p0*qi + p1**2*ti**2 + 2*p1*p2*ti**3 + 2*p1*p3*ti**4 + 2*p1*p4*ti**5 + 2*p1*p5*ti**6 - 2*p1*qi*ti + p2**2*ti**4 + 2*p2*p3*ti**5 + 2*p2*p4*ti**6 + 2*p2*p5*ti**7 - 2*p2*qi*ti**2 + p3**2*ti**6 + 2*p3*p4*ti**7 + 2*p3*p5*ti**8 - 2*p3*qi*ti**3 + p4**2*ti**8 + 2*p4*p5*ti**9 - 2*p4*qi*ti**4 + p5**2*ti**10 - 2*p5*qi*ti**5 + qi**2

2*p0 + 2*p1*ti + 2*p2*ti**2 + 2*p3*ti**3 + 2*p4*ti**4 + 2*p5*ti**5 - 2*qi

2*ti*(p0 + p1*ti + p2*ti**2 + p3*ti**3 + p4*ti**4 + p5*ti**5 - qi)

-4*ld*(2*p2*t1 - 2*p2*t2 + 3*p3*t1**2 - 3*p3*t2**2 + 4*p4*t1**3 - 4*p4*t2**3 + 5*p5*t1**4 - 5*p5*t2**4) + 2*p0*ti**2 + 2*p1*ti**3 + 2*p2*ti**4 + 2*p3*ti**5 + 2*p4*ti**6 + 2*p5*ti**7 - 2*qi*ti**2

-12*ld*(p2*t1**2 - p2*t2**2 + 2*p3*t1**3 - 2*p3*t2**3 + 3*p4*t1**4 - 3*p4*t2**4 + 4*p5*t1**5 - 4*p5*t2**5) + 2*p0*ti**3 + 2*p1*ti**4 + 2*p2*ti**5 + 2*p3*ti**6 + 2*p4*ti**7 + 2*p5*ti**8 - 2*qi*ti**3

-4*ld*(20*p2*t1**3 - 20*p2*t2**3 + 45*p3*t1**4 - 45*p3*t2**4 + 72*p4*t1**5 - 72*p4*t2**5 + 100*p5*t1**6 - 100*p5*t2**6)/5 + 2*p0*ti**4 + 2*p1*ti**5 + 2*p2*ti**6 + 2*p3*ti**7 + 2*p4*ti**8 + 2*p5*ti**9 - 2*qi*ti**4

-4*ld*(35*p2*t1**4 - 35*p2*t2**4 + 84*p3*t1**5 - 84*p3*t2**5 + 140*p4*t1**6 - 140*p4*t2**6 + 200*p5*t1**7 - 200*p5*t2**7)/7 + 2*p0*ti**5 + 2*p1*ti**6 + 2*p2*ti**7 + 2*p3*ti**8 + 2*p4*ti**9 + 2*p5*ti**10 - 2*qi*ti**5

