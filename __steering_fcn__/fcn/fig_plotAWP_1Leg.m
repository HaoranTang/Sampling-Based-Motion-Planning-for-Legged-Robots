function P = fig_plotAWP_1Leg(J,p)

umax = p.umax;
mu = p.mu;
scale = 5e-4;

U = zeros(2,4);
U(:,1) = [umax;umax];
U(:,2) = [-umax;umax];
U(:,3) = [-umax;-umax];
U(:,4) = [umax;-umax];

F = zeros(2,4);
for ii = 1:4
    F(:,ii) = J' \ U(:,ii);
end

Fplot = scale * [F,F(:,1)];

plot(Fplot(1,:),Fplot(2,:),'k-')


% YALMIP polytope
A = [[1 0;...
     -1 0;...
     0 1;...
     0 -1]*J';...
     1 -mu;
     -1 -mu];
b = [umax*[1;1;1;1];0;0];

P = Polyhedron(A,scale*b);

