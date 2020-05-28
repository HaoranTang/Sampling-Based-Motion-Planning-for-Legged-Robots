function infObs = fcn_get_infObs(flag)
% infObs = [pObs;slope]
% pObs: starting point of a segment of terrain
% slope: the slope of that terrain in [rad]

if flag == 0    % flat ground
    pObs = [[-0.1;0],[2.5;0]];
    slope = [0,0] * pi/180;
elseif flag == 1 % one obstacle
    pObs = [[-2;0],[0.5;0.2],[2.5;0],[5.0;0]];
    slope = [0,0,0,0] * pi/180;
elseif flag == 2 % parkour
    pObs = [[-0.6;0.3],[-0.32;0],[0.1;0.66],[0.63;0]];
    slope = [-16 0 0 0]/180*pi;
elseif flag == 3 % 3 jumps
    pObs = [[-0.6;0.3],[-0.32;0],[0.1;0.66],[0.4;0.5],[1.2;0]];
    slope = [0 0 0 0 0]/180*pi;
else
    pObs = [[-0.1;0],[0.8;0]];
    slope = [0,0] * pi/180;
end

infObs = [pObs;slope];
