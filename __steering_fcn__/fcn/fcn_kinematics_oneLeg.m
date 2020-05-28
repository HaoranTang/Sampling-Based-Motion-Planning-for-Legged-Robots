function [J,chain] = fcn_kinematics_oneLeg(X,p)

l1 = p.l1;
x = X(1);
z = X(2);
p_f = X(3:4);

p_hip = [x;z];

[J,q] = getJacobian(p_hip - p_f);

p_knee = p_hip + l1 * [cos(q(1));sin(q(1))];

chain = [p_f,p_knee,p_hip];










