function vel = fcn_get_vel(x,co,dir)

g = 9.81;
vx = dir * sqrt(-g/(2*co(3)));
slope = co(2)+2*co(3)*x;
vz = slope * vx;

vel = [vx;vz];


end