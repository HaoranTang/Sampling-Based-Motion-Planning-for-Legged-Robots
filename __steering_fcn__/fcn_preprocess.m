function P = fcn_preprocess(path,coeff)

N = size(path,2);               % number of node
P{1}.pos = path(:,1);       % [m]
P{1}.vin = [0;0];
P{1}.vout = get_vel(coeff(:,1));
P{1}.T = coeff(3,1);

for ii = 2:N-1
    P{ii}.pos = path(:,ii);
    [~,P{ii}.vin] = get_vel(coeff(:,ii-1));
    P{ii}.vout = get_vel(coeff(:,ii));
    P{ii}.T = coeff(3,ii);
end
P{N}.pos = path(:,ii);
[~,P{N}.vin] = get_vel(coeff(:,N-1));


end

function [Vout,Vin_next] = get_vel(co)

g = 9.81;

% coef = [vel th time dir(1-right,2-left)]
V = co(1)/100;
th = co(2);
time = co(3);
dir = co(4);

if dir == 1
    vx_out = V*cos(th);
    vz_out = V*sin(th);
else
    vx_out = -V*cos(th);
    vz_out = V*sin(th);
end
vx_in = vx_out;
vz_in = vz_out - time * g;

Vout = [vx_out;vz_out];
Vin_next = [vx_in;vz_in];

end