function [PL_hat,PR_hat,FWP] = fig_plotAWP_2Legs(JL,JR,X,p)

umax = p.umax;
mu = p.mu;
scale = p.scale;

p_COM = X(1:2);
pfL = X(4:5);
pfR = X(6:7);

U = zeros(2,4);
U(:,1) = [umax;umax];
U(:,2) = [-umax;umax];
U(:,3) = [-umax;-umax];
U(:,4) = [umax;-umax];

FL = zeros(2,4);
FR = zeros(2,4);
for ii = 1:4
    FL(:,ii) = JL' \ U(:,ii);
    FR(:,ii) = JR' \ U(:,ii);
end

FL = [FL,FL(:,1)];
FR = [FR,FR(:,1)];
FplotL = scale * FL + repmat(pfL,[1,length(FL)]);
FplotR = scale * FR + repmat(pfR,[1,length(FR)]);

plot(FplotL(1,:),FplotL(2,:),'k-')
plot(FplotR(1,:),FplotR(2,:),'k-')


% YALMIP polytope
AL = [[1 0;...
     -1 0;...
     0 1;...
     0 -1]*JL';...
     1 -mu;
     -1 -mu];
bL = [umax*[1;1;1;1];0;0];

PL = Polyhedron(AL,bL);
PL_hat = fcn_P2P_hat(PL,pfL - p_COM);

AR = [[1 0;...
     -1 0;...
     0 1;...
     0 -1]*JR';...
     1 -mu;
     -1 -mu];
bR = [umax*[1;1;1;1];0;0];

PR = Polyhedron(AR,bR);
PR_hat = fcn_P2P_hat(PR,pfR - p_COM);


FWP = plus(PL_hat,PR_hat);

plot(scale*PL_hat + [pfL;0])
plot(scale*PR_hat + [pfR;0])
plot(scale * FWP)

end

function P_hat = fcn_P2P_hat(P,r)
% P is the polyhedron for forces R^2
% P_hat is the polyhedron for wrench R^3

b = P.V;
len = length(b);
b_hat = zeros(len,3);
for ii = 1:len
    b_hat(ii,1:2) = b(ii,:);
    b_hat(ii,3) = wedgeMap(r,b(ii,:));
end
P_hat = Polyhedron('V',b_hat);

end





