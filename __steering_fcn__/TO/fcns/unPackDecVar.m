function [t,x,u] = unPackDecVar(z,pack)
%
% This function unpacks the decision variables for
% trajectory optimization into the time (t),
% state (x), and control (u) matricies
%
% INPUTS:
%   z = column vector of 2 + nTime*(nState+nControl) decision variables
%   pack = details about how to convert z back into t,x, and u
%       .nTime
%       .nState
%       .nControl
%
% OUTPUTS:
%   t = [1, nTime] = time vector (grid points)
%   x = [nState, nTime] = state vector at each grid point
%   u = [nControl, nTime] = control vector at each grid point
%

nTime = pack.nTime;
nState = pack.nState;
nControl = pack.nControl;

t = linspace(z(1),z(2),nTime);

x = z(pack.xIdx);
u = z(pack.uIdx);

% make sure x and u are returned as vectors, [nState,nTime] and
% [nControl,nTime]
x = reshape(x,nState,nTime);
u = reshape(u,nControl,nTime);

end