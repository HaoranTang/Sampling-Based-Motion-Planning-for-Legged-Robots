%% Animate solution at every iteration. 
% See fmincon documentation for more details on output functions
function stop = outfun(z, optimValues, state,pack,p)
    switch state
        case 'iter'
%             animate(z,pack,p,0.00); 
    end
    stop = 0;
end