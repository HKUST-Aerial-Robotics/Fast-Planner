#include "pose_utils.h"

// Rotation ---------------------

mat ypr_to_R(const colvec& ypr)
{
  double c, s;
  mat Rz = zeros<mat>(3,3);
  double y = ypr(0);
  c = cos(y);
  s = sin(y);
  Rz(0,0) =  c;
  Rz(1,0) =  s;
  Rz(0,1) = -s;
  Rz(1,1) =  c;
  Rz(2,2) =  1; 

  mat Ry = zeros<mat>(3,3);
  double p = ypr(1);
  c = cos(p);
  s = sin(p);
  Ry(0,0) =  c;
  Ry(2,0) = -s;
  Ry(0,2) =  s;
  Ry(2,2) =  c;
  Ry(1,1) =  1; 

  mat Rx = zeros<mat>(3,3);
  double r = ypr(2);
  c = cos(r);
  s = sin(r);
  Rx(1,1) =  c;
  Rx(2,1) =  s;
  Rx(1,2) = -s;
  Rx(2,2) =  c;
  Rx(0,0) =  1; 

  mat R = Rz*Ry*Rx;  
  return R;
}

mat yaw_to_R(double yaw)
{
  mat R = zeros<mat>(2,2);
  double c = cos(yaw);
  double s = sin(yaw);
  R(0,0) =  c;
  R(1,0) =  s;
  R(0,1) = -s;
  R(1,1) =  c;
  return R;
}

colvec R_to_ypr(const mat& R)
{
  colvec n = R.col(0);
  colvec o = R.col(1);
  colvec a = R.col(2);

  colvec ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0)*cos(y)+n(1)*sin(y));
  double r = atan2(a(0)*sin(y)-a(1)*cos(y), -o(0)*sin(y)+o(1)*cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr;
}

mat quaternion_to_R(const colvec& q)
{
  double n = norm(q, 2);
  colvec nq = q / n;

  double w = nq(0); 
  double x = nq(1); 
  double y = nq(2);
  double z = nq(3);
  double w2 = w*w;
  double x2 = x*x;
  double y2 = y*y; 
  double z2 = z*z;
  double xy = x*y; 
  double xz = x*z; 
  double yz = y*z;
  double wx = w*x; 
  double wy = w*y; 
  double wz = w*z;

  mat R = zeros<mat>(3,3);
  R(0,0) = w2+x2-y2-z2;
  R(1,0) = 2*(wz + xy);
  R(2,0) = 2*(xz - wy);
  R(0,1) = 2*(xy - wz);
  R(1,1) = w2-x2+y2-z2;
  R(2,1) = 2*(wx + yz);
  R(0,2) = 2*(wy + xz);
  R(1,2) = 2*(yz - wx);
  R(2,2) = w2-x2-y2+z2;
  return R;
}

colvec R_to_quaternion(const mat& R)
{
  colvec q(4);
  double  tr = R(0,0) + R(1,1) + R(2,2);
  if (tr > 0) 
  { 
    double S = sqrt(tr + 1.0) * 2; 
    q(0) = 0.25 * S;
    q(1) = (R(2,1) - R(1,2)) / S;
    q(2) = (R(0,2) - R(2,0)) / S; 
    q(3) = (R(1,0) - R(0,1)) / S; 
  } 
  else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) 
  { 
    double S = sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2; 
    q(0) = (R(2,1) - R(1,2)) / S;
    q(1) = 0.25 * S;
    q(2) = (R(0,1) + R(1,0)) / S; 
    q(3) = (R(0,2) + R(2,0)) / S; 
  } 
  else if (R(1,1) > R(2,2)) 
  { 
    double S = sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2; 
    q(0) = (R(0,2) - R(2,0)) / S;
    q(1) = (R(0,1) + R(1,0)) / S; 
    q(2) = 0.25 * S;
    q(3) = (R(1,2) + R(2,1)) / S; 
  } 
  else 
  { 
    double S = sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2; 
    q(0) = (R(1,0) - R(0,1)) / S;
    q(1) = (R(0,2) + R(2,0)) / S;
    q(2) = (R(1,2) + R(2,1)) / S;
    q(3) = 0.25 * S;
  }
  return q;
}

colvec quaternion_mul(const colvec& q1, const colvec& q2)
{
  double a1 = q1(0);
  double b1 = q1(1);
  double c1 = q1(2);
  double d1 = q1(3);
    
  double a2 = q2(0);
  double b2 = q2(1);
  double c2 = q2(2);
  double d2 = q2(3);
    
  colvec q3(4);
  q3(0) = a1*a2 - b1*b2 - c1*c2 - d1*d2;
  q3(1) = a1*b2 + b1*a2 + c1*d2 - d1*c2;
  q3(2) = a1*c2 - b1*d2 + c1*a2 + d1*b2;
  q3(3) = a1*d2 + b1*c2 - c1*b2 + d1*a2;
  return q3;
}

colvec quaternion_inv(const colvec& q)
{
  colvec q2(4);
  q2(0) =  q(0);
  q2(1) = -q(1);
  q2(2) = -q(2);
  q2(3) = -q(3);    
  return q2;
}

// General Pose Update ----------

colvec pose_update(const colvec& X1, const colvec& X2)
{
  mat R1 = ypr_to_R(X1.rows(3,5));
  mat R2 = ypr_to_R(X2.rows(3,5));
  mat R3 = R1 * R2;

  colvec X3xyz = X1.rows(0,2) + R1*X2.rows(0,2);
  colvec X3ypr = R_to_ypr(R3);

  colvec X3 = join_cols(X3xyz, X3ypr);
  return X3;
}

colvec pose_inverse(const colvec& X)
{
  mat R = ypr_to_R(X.rows(3,5));
  colvec n = R.col(0);
  colvec o = R.col(1);
  colvec a = R.col(2);

  colvec XIxyz = -trans(R) * (X.rows(0,2));
  colvec XIypr(3);
  double XIy = atan2(o(0), n(0));
  double XIp = atan2(-a(0), n(0)*cos(XIy)+o(0)*sin(XIy));
  double XIr = atan2(n(2)*sin(XIy)-o(2)*cos(XIy), -n(1)*sin(XIy)+o(1)*cos(XIy));
  XIypr(0) = XIy;
  XIypr(1) = XIp;
  XIypr(2) = XIr;

  colvec XI = join_cols(XIxyz, XIypr);
  return XI;
}

colvec pose_update_2d(const colvec& X1, const colvec& X2)
{
  mat R = yaw_to_R(X1(2));
  colvec X3(3);
  X3.rows(0,1) = R * X2.rows(0,1) + X1.rows(0,1);
  X3(2)        = X1(2) + X2(2);
  return X3;
}

colvec pose_inverse_2d(const colvec& X)
{
  double c = cos(X(2));
  double s = sin(X(2));
  colvec XI(3);
  XI(0) = -X(0) * c - X(1) * s;
  XI(1) =  X(0) * s - X(1) * c;
  XI(2) = -X(2);
  return XI;
}


// For Pose EKF ----------------------

mat Jplus1(const colvec& X1, const colvec& X2)
{
  colvec X3 = pose_update(X1,X2);
  mat R1 = ypr_to_R(X1.rows(3,5));
  mat R2 = ypr_to_R(X2.rows(3,5));
  mat R3 = ypr_to_R(X3.rows(3,5));    
  colvec o1 = R1.col(1);
  colvec a1 = R1.col(2);    
  colvec o2 = R2.col(1);
  colvec a2 = R2.col(2);    

  mat I = eye<mat>(3,3);
  mat Z = zeros<mat>(3,3);
    
  double Marr[9]  = {  -(X3(2-1)-X1(2-1))   ,                        (X3(3-1)-X1(3-1))*cos(X1(4-1))                             ,   a1(1-1)*X2(2-1)-o1(1-1)*X2(3-1)   ,   
                        X3(1-1)-X1(1-1)     ,                        (X3(3-1)-X1(3-1))*sin(X1(4-1))                             ,   a1(2-1)*X2(2-1)-o1(2-1)*X2(3-1)   ,  
                       0          , -X2(1-1)*cos(X1(5-1))-X2(2-1)*sin(X1(5-1))*sin(X1(6-1))-X2(3-1)*sin(X1(5-1))*cos(X1(6-1))   ,   a1(3-1)*X2(2-1)-o1(3-1)*X2(3-1)   };
  mat M(3,3);
  for (int i =0; i < 9; i++)
    M(i) = Marr[i];
  M = trans(M);
             
  double K1arr[9] = {        1          ,   sin(X3(5-1))*sin(X3(4-1)-X1(4-1))/cos(X3(5-1))   ,   ( o2(1-1)*sin(X3(6-1))+a2(1-1)*cos(X3(6-1)) )/cos(X3(5-1))   , 
                             0          ,              cos(X3(4-1)-X1(4-1))                  ,                 -cos(X1(5-1))*sin(X3(4-1)-X1(4-1))             , 
                             0          ,         sin(X3(4-1)-X1(4-1))/cos(X3(5-1))          ,         cos(X1(5-1))*cos(X3(4-1)-X1(4-1))/cos(X3(5-1))         };
  mat K1(3,3);
  for (int i =0; i < 9; i++)
    K1(i) = K1arr[i];
  K1 = trans(K1);
      
  mat J1 = join_cols ( join_rows(I, M) , join_rows(Z,K1) );
  return J1;
}

mat Jplus2(const colvec& X1, const colvec& X2)
{
  colvec X3 = pose_update(X1,X2);
  mat R1 = ypr_to_R(X1.rows(3,5));
  mat R2 = ypr_to_R(X2.rows(3,5));
  mat R3 = ypr_to_R(X3.rows(3,5));    
  colvec o1 = R1.col(1);
  colvec a1 = R1.col(2);    
  colvec o2 = R2.col(1);
  colvec a2 = R2.col(2);    

  mat Z = zeros<mat>(3,3);
              
  double K2arr[9] = {     cos(X2(5-1))*cos(X3(6-1)-X2(6-1))/cos(X3(5-1))          ,        sin(X3(6-1)-X2(6-1))/cos(X3(5-1))            ,   0   ,
                              -cos(X2(5-1))*sin(X3(6-1)-X2(6-1))                  ,              cos(X3(6-1)-X2(6-1))                   ,   0   ,
                     ( a1(1-1)*cos(X3(4-1))+a1(2-1)*sin(X3(4-1)) )/cos(X3(5-1))   ,    sin(X3(5-1))*sin(X3(6-1)-X2(6-1))/cos(X3(5-1))   ,   1   };
  mat K2(3,3);
  for (int i =0; i < 9; i++)
    K2(i) = K2arr[i];
  K2 = trans(K2);
      
  mat J2 = join_cols ( join_rows(R1,Z) , join_rows(Z,K2) );
  return J2;
}


// For IMU EKF ----------------------

colvec state_update(const colvec& X, const colvec& U, double dt)
{
  double ro = X(3);
  double pt = X(4);
  double ya = X(5);

  colvec ypr(3);
  ypr(0) = ya;
  ypr(1) = pt;
  ypr(2) = ro;
  mat R = ypr_to_R(ypr);

  mat M(3,3);
  M(0,0) = 1; M(0,1) =  0;       M(0,2) = -sin(pt);
  M(1,0) = 0; M(1,1) =  cos(ro); M(1,2) =  cos(pt)*sin(ro);
  M(2,0) = 0; M(2,1) = -sin(ro); M(2,2) =  cos(pt)*cos(ro);

  colvec Xt(9);
  Xt.rows(0,2) = X.rows(0,2) + X.rows(6,8)*dt + R*U.rows(0,2)*dt*dt/2;
  Xt.rows(3,5) = X.rows(3,5) + inv(M)*U.rows(3,5)*dt;
  Xt.rows(6,8) = X.rows(6,8) + R*U.rows(0,2)*dt;

  return Xt;
}

mat jacobianF(const colvec& X, const colvec& U, double dt)
{
  double x  = X(0);
  double y  = X(1);
  double z  = X(2);
  double ro = X(3);
  double pt = X(4);
  double ya = X(5);
  double vx = X(6);
  double vy = X(7);
  double vz = X(8);
  double ax = U(0);
  double ay = U(1);
  double az = U(2);
  double wx = U(3);
  double wy = U(4);
  double wz = U(5);

  mat F(9,9);

  F(0,0) = 1;
  F(0,1) = 0;
  F(0,2) = 0;
  F(0,3) = (dt*dt*(ay*(sin(ro)*sin(ya) + cos(ro)*cos(ya)*sin(pt)) + az*(cos(ro)*sin(ya) - cos(ya)*sin(pt)*sin(ro))))/2,
  F(0,4) = (dt*dt*(az*cos(pt)*cos(ro)*cos(ya) - ax*cos(ya)*sin(pt) + ay*cos(pt)*cos(ya)*sin(ro)))/2; 
  F(0,5) = -(dt*dt*(ay*(cos(ro)*cos(ya) + sin(pt)*sin(ro)*sin(ya)) - az*(cos(ya)*sin(ro) - cos(ro)*sin(pt)*sin(ya)) + ax*cos(pt)*sin(ya)))/2;
  F(0,6) = dt;
  F(0,7) = 0;
  F(0,8) = 0;

  F(1,0) = 0;
  F(1,1) = 1;
  F(1,2) = 0;
  F(1,3) = -(dt*dt*(ay*(cos(ya)*sin(ro) - cos(ro)*sin(pt)*sin(ya)) + az*(cos(ro)*cos(ya) + sin(pt)*sin(ro)*sin(ya))))/2;
  F(1,4) = (dt*dt*(az*cos(pt)*cos(ro)*sin(ya) - ax*sin(pt)*sin(ya) + ay*cos(pt)*sin(ro)*sin(ya)))/2;
  F(1,5) = (dt*dt*(az*(sin(ro)*sin(ya) + cos(ro)*cos(ya)*sin(pt)) - ay*(cos(ro)*sin(ya) - cos(ya)*sin(pt)*sin(ro)) + ax*cos(pt)*cos(ya)))/2;
  F(1,6) = 0;
  F(1,7) = dt;
  F(1,8) = 0;

  F(2,0) = 0;
  F(2,1) = 0;
  F(2,2) = 1;
  F(2,3) = (dt*dt*(ay*cos(pt)*cos(ro) - az*cos(pt)*sin(ro)))/2,
  F(2,4) = -(dt*dt*(ax*cos(pt) + az*cos(ro)*sin(pt) + ay*sin(pt)*sin(ro)))/2,
  F(2,5) = 0;
  F(2,6) = 0;
  F(2,7) = 0;
  F(2,8) = dt;

  F(3,0) = 0;
  F(3,1) = 0;
  F(3,2) = 0;
  F(3,3) = 1 - dt*((wz*(cos(pt - ro)/2 - cos(pt + ro)/2))/cos(pt) - (wy*(sin(pt - ro)/2 + sin(pt + ro)/2))/cos(pt));
  F(3,4) = dt*((wz*(cos(pt - ro)/2 + cos(pt + ro)/2))/cos(pt) - (wy*(sin(pt - ro)/2 - sin(pt + ro)/2))/cos(pt) + 
           (wy*sin(pt)*(cos(pt - ro)/2 - cos(pt + ro)/2))/(cos(pt)*cos(pt)) + (wz*sin(pt)*(sin(pt - ro)/2 + sin(pt + ro)/2))/(cos(pt)*cos(pt)));
  F(3,5) = 0;
  F(3,6) = 0;
  F(3,7) = 0;
  F(3,8) = 0;

  F(4,0) = 0;
  F(4,1) = 0;
  F(4,2) = 0;
  F(4,3) = -dt*(wz*cos(ro) + wy*sin(ro));
  F(4,4) = 1;
  F(4,5) = 0;
  F(4,6) = 0;
  F(4,7) = 0;
  F(4,8) = 0;

  F(5,0) = 0;
  F(5,1) = 0;
  F(5,2) = 0;
  F(5,3) = dt*((wy*cos(ro))/cos(pt) - (wz*sin(ro))/cos(pt));
  F(5,4) = dt*((wz*cos(ro)*sin(pt))/(cos(pt)*cos(pt)) + (wy*sin(pt)*sin(ro))/(cos(pt)*cos(pt)));
  F(5,5) = 1;
  F(5,6) = 0;
  F(5,7) = 0;
  F(5,8) = 0;

  F(6,0) = 0;
  F(6,1) = 0;
  F(6,2) = 0;
  F(6,3) = dt*(ay*(sin(ro)*sin(ya) + cos(ro)*cos(ya)*sin(pt)) + az*(cos(ro)*sin(ya) - cos(ya)*sin(pt)*sin(ro)));
  F(6,4) = dt*(az*cos(pt)*cos(ro)*cos(ya) - ax*cos(ya)*sin(pt) + ay*cos(pt)*cos(ya)*sin(ro));
  F(6,5) = -dt*(ay*(cos(ro)*cos(ya) + sin(pt)*sin(ro)*sin(ya)) - az*(cos(ya)*sin(ro) - cos(ro)*sin(pt)*sin(ya)) + ax*cos(pt)*sin(ya));
  F(6,6) = 1;
  F(6,7) = 0;
  F(6,8) = 0;

  F(7,0) = 0;
  F(7,1) = 0;
  F(7,2) = 0;
  F(7,3) = -dt*(ay*(cos(ya)*sin(ro) - cos(ro)*sin(pt)*sin(ya)) + az*(cos(ro)*cos(ya) + sin(pt)*sin(ro)*sin(ya)));
  F(7,4) = dt*(az*cos(pt)*cos(ro)*sin(ya) - ax*sin(pt)*sin(ya) + ay*cos(pt)*sin(ro)*sin(ya));
  F(7,5) = dt*(az*(sin(ro)*sin(ya) + cos(ro)*cos(ya)*sin(pt)) - ay*(cos(ro)*sin(ya) - cos(ya)*sin(pt)*sin(ro)) + ax*cos(pt)*cos(ya));
  F(7,6) = 0;
  F(7,7) = 1;
  F(7,8) = 0;

  F(8,0) = 0;
  F(8,1) = 0;
  F(8,2) = 0;
  F(8,3) = dt*(ay*cos(pt)*cos(ro) - az*cos(pt)*sin(ro));     
  F(8,4) = -dt*(ax*cos(pt) + az*cos(ro)*sin(pt) + ay*sin(pt)*sin(ro));
  F(8,5) = 0;
  F(8,6) = 0;
  F(8,7) = 0;
  F(8,8) = 1;
   
  return F;
}

mat jacobianU(const colvec& X, const colvec& U, double dt)
{
  double x  = X(0);
  double y  = X(1);
  double z  = X(2);
  double ro = X(3);
  double pt = X(4);
  double ya = X(5);
  double vx = X(6);
  double vy = X(7);
  double vz = X(8);
  double ax = U(0);
  double ay = U(1);
  double az = U(2);
  double wx = U(3);
  double wy = U(4);
  double wz = U(5);

  mat G(9,6);

  G(0,0) = (dt*dt*cos(pt)*cos(ya))/2;
  G(0,1) = -(dt*dt*(cos(ro)*sin(ya) - cos(ya)*sin(pt)*sin(ro)))/2;
  G(0,2) = (dt*dt*(sin(ro)*sin(ya) + cos(ro)*cos(ya)*sin(pt)))/2;
  G(0,3) = 0;
  G(0,4) = 0;
  G(0,5) = 0;

  G(1,0) = (dt*dt*cos(pt)*sin(ya))/2;
  G(1,1) = (dt*dt*(cos(ro)*cos(ya) + sin(pt)*sin(ro)*sin(ya)))/2;
  G(1,2) = -(dt*dt*(cos(ya)*sin(ro) - cos(ro)*sin(pt)*sin(ya)))/2;
  G(1,3) = 0;
  G(1,4) = 0;
  G(1,5) = 0;

  G(2,0) = -(dt*dt*sin(pt))/2;
  G(2,1) = (dt*dt*cos(pt)*sin(ro))/2;
  G(2,2) = (dt*dt*cos(pt)*cos(ro))/2;
  G(2,3) = 0;
  G(2,4) = 0;
  G(2,5) = 0;

  G(3,0) = 0;
  G(3,1) = 0;
  G(3,2) = 0;
  G(3,3) = dt;
  G(3,4) = (dt*(cos(pt - ro)/2 - cos(pt + ro)/2))/cos(pt);
  G(3,5) = (dt*(sin(pt - ro)/2 + sin(pt + ro)/2))/cos(pt);

  G(4,0) = 0;
  G(4,1) = 0;
  G(4,2) = 0;
  G(4,3) = 0;
  G(4,4) = dt*cos(ro);
  G(4,5) = -dt*sin(ro);

  G(5,0) = 0;
  G(5,1) = 0;
  G(5,2) = 0;
  G(5,3) = 0;
  G(5,4) = (dt*sin(ro))/cos(pt);
  G(5,5) = (dt*cos(ro))/cos(pt);

  G(6,0) = dt*cos(pt)*cos(ya);
  G(6,1) = -dt*(cos(ro)*sin(ya) - cos(ya)*sin(pt)*sin(ro)); 
  G(6,2) = dt*(sin(ro)*sin(ya) + cos(ro)*cos(ya)*sin(pt));
  G(6,3) = 0;
  G(6,4) = 0;
  G(6,5) = 0;

  G(7,0) = dt*cos(pt)*sin(ya);
  G(7,1) = dt*(cos(ro)*cos(ya) + sin(pt)*sin(ro)*sin(ya));
  G(7,2) = -dt*(cos(ya)*sin(ro) - cos(ro)*sin(pt)*sin(ya));
  G(7,3) = 0;
  G(7,4) = 0;
  G(7,5) = 0;

  G(8,0) = -dt*sin(pt);
  G(8,1) = dt*cos(pt)*sin(ro); 
  G(8,2) = dt*cos(pt)*cos(ro);
  G(8,3) = 0;
  G(8,4) = 0;
  G(8,5) = 0;

  return G;
}

colvec state_measure(const colvec& X)
{
  colvec Z = X.rows(0,5);
  return Z;
}

mat jacobianH()
{
  mat H = zeros<mat>(6,9);
  H.cols(0,5) = eye<mat>(6,6);

  return H;
}

