function plot_para(co,P)

scale = 1e-1;

fcn_para = @(x,co)co(1)+co(2)*x+co(3)*x.^2;

hold on

N = length(P);
for ii = 1:N-1
    x = linspace(P{ii}.pos(1),P{ii+1}.pos(1),101);
    y = fcn_para(x,co(:,ii));
    plot(x,y,'k')
    
    vin = P{ii}.vin;
    vout = P{ii}.vout;
    plot(P{ii}.pos(1)+scale*[0,vin(1)],P{ii}.pos(2)+scale*[0,vin(2)],'r')
    plot(P{ii}.pos(1)+scale*[0,vout(1)],P{ii}.pos(2)+scale*[0,vout(2)],'b')
end

axis equal

end