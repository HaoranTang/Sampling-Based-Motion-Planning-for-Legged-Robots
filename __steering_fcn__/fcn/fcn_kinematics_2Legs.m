function [JL,JR,chain,qL,qR] = fcn_kinematics_2Legs(X,p)

L = p.L;
l1 = p.l1;

x = X(1);
z = X(2);
th = X(3);
p_fR = X(4:5);
p_fL = X(6:7);

p_COM = [x;z];
p_hipR = p_COM + L/2 * [cos(th);sin(th)];
p_hipL = p_COM - L/2 * [cos(th);sin(th)];


[JL,qL] = getJacobian(p_hipL - p_fL);
[JR,qR] = getJacobian(p_hipR - p_fR);

p_kneeR = p_hipR + l1*[cos(qR(1));sin(qR(1))];
p_kneeL = p_hipL + l1*[cos(qL(1));sin(qL(1))];

% chain of joints for plottting
chain = [p_fL,p_kneeL,p_hipL,p_hipR,p_kneeR,p_fR];



