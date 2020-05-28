function [grid0,JJT0,A0,b0] = fcn_generate_grid(infObs,Nr,Nth)

Nobs = length(infObs) - 1;

% grid with slope 0
slope0 = 0; % [rad]
grid0 = fcn_Initialize_grid_w_slope(Nr,Nth,slope0);
JJT0 = getJsec(grid0);
% get the geometry of the grid
[A0,b0] = getAb(grid0);

for ii = 1:Nobs
    if infObs(3,ii) == 0
        fprintf('Obstacle %d has slope 0. \n',ii)
        
        % Initialize grid
        grid(:,:,ii) = grid0;
        fprintf('grid generation complete!\n')
        
        % get the minmum emcompassing ellipsoid
        JJT(:,:,ii) = JJT0;
        fprintf('JJT Estimation complete!\n')
        
        % get the geometry of the grid
        A(:,:,ii) = A0;
        b(:,ii) = b0;
        fprintf('A,b Geometry complete!\n')
    else
        fprintf('Obstacle %d has slope %0.2 degree\n',ii,infObs(3,ii)*180/pi)
        
        % Initialize grid
        slope = infObs(3,ii);
        grid_slope = fcn_Initialize_grid_w_slope(Nr,Nth,slope);
        grid(:,:,ii) = grid_slope;
        fprintf('grid generation complete!\n')
        
        % get the minmum emcompassing ellipsoid
        JJT(:,:,ii) = getJsec(grid_slope);
        fprintf('JJT Estimation complete!\n')
        
        % get the geometry of the grid
        [A0,b0] = getAb(grid_slope);
        A(:,:,ii) = A0;
        b(:,ii) = b0;
        fprintf('A,b Geometry complete!\n')
    end
end
