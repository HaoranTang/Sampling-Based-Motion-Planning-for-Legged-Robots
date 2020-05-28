function [z,pack] = packDecVar(t,x,u)
%
% This function collapses the time (t), state (x)
% and control (u) matricies into a single vector
%
% INPUTS:
%   t = [1, nTime] = time vector (grid points)
%   x = [nState, nTime] = state vector at each grid point
%   u = [nControl, nTime] = control vector at each grid point
%
% OUTPUTS:
%   z = column vector of 2 + nTime*(nState+nControl) decision variables
%   pack = details about how to convert z back into t,x, and u
%       .nTime
%       .nState
%       .nControl
%

nTime = length(t);
nState = size(x,1);
nControl = size(u,1);

tSpan = [t(1); t(end)];
xCol = reshape(x, nState*nTime, 1);
uCol = reshape(u, nControl*nTime, 1);

indz = reshape(2+(1:numel(u)+numel(x)),nState+nControl,nTime);

% index of time, state, control variables in the decVar vector
tIdx = 1:2;
xIdx = indz(1:nState,:);
uIdx = indz(nState+(1:nControl),:);

% decision variables
% variables are indexed so that the defects gradients appear as a banded
% matrix
z = zeros(2+numel(indz),1);
z(tIdx(:),1) = tSpan;
z(xIdx(:),1) = xCol;
z(uIdx(:),1) = uCol;

pack.nTime = nTime;
pack.nState = nState;
pack.nControl = nControl;
pack.tIdx = tIdx;
pack.xIdx = xIdx;
pack.uIdx = uIdx;

end