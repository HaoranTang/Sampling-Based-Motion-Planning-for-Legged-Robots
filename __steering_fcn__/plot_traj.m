function plot_traj(P)

g = 9.81;
scale = 1.4e-1;

N = length(P);
x = [];
z = [];
for ii = 1:N-1
    T = P{ii}.T;
    pos = P{ii}.pos;
    vin = P{ii}.vin;
    vout = P{ii}.vout;
    
    % vin and vout
    plot(pos(1)+[0,scale*vin(1)],pos(2)+[0,scale*vin(2)],'r','linewidth',1)
    plot(pos(1)+[0,scale*vout(1)],pos(2)+[0,scale*vout(2)],'b','linewidth',1)
    
    % the parabola
    t = linspace(0,T,1001);
    x = [x,pos(1)+vout(1)*t];
    z = [z,pos(2)+vout(2)*t - 1/2*g*t.^2];
end

plot(x,z,'k--','linewidth',1)



end