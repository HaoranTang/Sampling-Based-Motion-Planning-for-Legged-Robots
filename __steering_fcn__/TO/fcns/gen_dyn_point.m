clear;
clc;

syms x y dx dy ddx ddy real
syms ux uy real
syms m g real

%%% unit vector 
i = sym([1;0]);
j = sym([0;1]);

%%% state vector
z = [x;y;dx;dy];
dz = [dx;dy;ddx;ddy];

%%% Kinematics
derivative = @(f)(jacobian(f,z)*dz);    %chain rule

pos = x * i + y * j;

dpos = derivative(pos);

ddp = derivative(dpos);

%%% Horizontal force balance
sumForces = ux;
sumInertial = m * dot(ddp,i);
eqn1 = sumForces - sumInertial;

%%% Vertical force balance
sumForces = uy - m * g;
sumInertial = m * dot(ddp,j);
eqn2 = sumForces - sumInertial;

%%% Write out dynamics in matrix form
[AA,bb] = equationsToMatrix([eqn1,eqn2],ddp);
dyn = simplify(AA\bb);

%%% Generate function for dynamics
matlabFunction(dyn(1),dyn(2),...
    'file','autoGen_pointMassDynamics.m',...
    'vars',{ux,uy,m,g},...
    'outputs',{'ddx','ddy'});




