
syms x z real
syms c0 c1 c2 real
syms vx vz g t real
syms x1 z1 x2 z2 real

% z as a function of x
Z1 = c0 + c1 * x + c2 * x^2;

t = (x - x1) / vx;
Z2 = z1 + vz * t - 1/2 * g * t^2;

collect(Z2);

% c0 = z1 - (vz*x1)/vx - (g*x1^2)/(2*vx^2)
% c1 = vz/vx + (g*x1)/vx^2
% c2 = - g/(2*vx^2)
% |vz/vx| = q >= 1/mu
% q = c1 + 2*c2 * x1
% q = (z1-c0)/x1 + x1 * c2

q = c1 + 2*c2 * x1;
q = (z1-c0)/x1 + x1 * c2;

V2 = (1+q^2) * (-g/(2*c2))
