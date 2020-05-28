% Derive EoM of 2D planar quadruped for backflip
% 
% Model: single rigid body (SRB)
% ground reaction force (GRF) applied on the ground


clear;
clc;

%%%% Create symbolic variables:
syms x y th dx dy dth ddx ddy ddth real
syms u1x u1y u2x u2y real
syms m J g L real

%%% unit vector for inertial frame
i = sym([1;0;0]);
j = sym([0;1;0]);
k = sym([0;0;1]);

%%% unit vector for body frame
ib = cos(th)*i + sin(th)*j;
jb = -sin(th)*i + cos(th)*j;

%%% state vector
z = [x;y;th;dx;dy;dth];
dz = [dx;dy;dth;ddx;ddy;ddth];

%%% force vector
u1 = u1x * i + u1y *j;
u2 = u2x * i + u2y *j;
Fg = -m*g*j; 

%%% Kinematics
derivative = @(f)(jacobian(f,z)*dz);    %chain rule

q = x * i + y * j + th * k;     % q = [x,y,th]
dq = derivative(q);
ddq = derivative(dq);
pc = x * i + y * j;
ph1 = pc - L/2 * ib;    % left hip
ph2 = pc + L/2 * ib;    % right hip
pf1 = -L/2 * i;         % left foot
pf2 = L/2 * i;          % right foot

%%% Force balance
sumForces = u1 + u2 + Fg;
sumInertial = m * [ddx;ddy;0];
eqn1 = dot(sumForces-sumInertial,i);
eqn2 = dot(sumForces-sumInertial,j);

%%% Angular momentum balance
sumTorques = cross(pf1-pc,u1) + cross(pf2-pc,u2);
angMomentum = J * ddth * k;
eqn3 = simplify(dot(sumTorques-angMomentum,k));

%%% Write out dynamics in matrix form
eqns = [eqn1;eqn2;eqn3];
[AA,bb] = equationsToMatrix(eqns,ddq);
dyn = simplify(AA\bb);

%%% Generate function for dynamics
matlabFunction(dyn(1),dyn(2),dyn(3),...
    'file','autoGen_robotDynamics.m',...
    'vars',{x,y,u1x,u1y,u2x,u2y,m,J,g,L},...
    'outputs',{'ddx','ddy','ddth'});




