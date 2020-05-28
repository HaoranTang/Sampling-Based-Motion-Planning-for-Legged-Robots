function [A,b] = getAb(grid)

N = length(grid);

A = zeros(0,2);
b = zeros(0,1);

for i_N = 1:N
    idx = 3*i_N - 2 : 3*i_N;
    [A(idx,:),b(idx)] = getAbCell(grid{i_N});
end

end

function [A,b] = getAbCell(g)

if size(g,2) ~= 2
    g = g';
elseif ~ispolycw(g(:,1),g(:,2))
    [x1, y1] = poly2cw(g(:,1), g(:,2));
    g(:,1) = x1;
    g(:,2) = y1;
end
p1 = g(1,:)';p2 = g(2,:)';p3 = g(3,:)';
v12 = p2 - p1;
v23 = p3 - p2;
v31 = p1 - p3;


A = zeros(3,2);
b = zeros(3,1);
A(1,:) = (rot(pi/2) * v12)';
b(1) = A(1,:) * p1;
A(2,:) = (rot(pi/2) * v23)';
b(2) = A(2,:) * p2;
A(3,:) = (rot(pi/2) * v31)';
b(3) = A(3,:) * p3;

end


function R = rot(th)
c = cos(th);
s = sin(th);

R = [c -s;s c];

end