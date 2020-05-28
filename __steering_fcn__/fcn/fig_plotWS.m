function fig_plotWS(X_ws,c_color)

if nargin < 2
    c_color = 'red';
end

len = length(X_ws);


for ii = 1:len
    X = X_ws(ii,:);
    plot3(X(1),X(2),X(3),'color',c_color,'marker','o')

end

xlabel('x [m]')
ylabel('z [m]')
zlabel('th [rad]')

