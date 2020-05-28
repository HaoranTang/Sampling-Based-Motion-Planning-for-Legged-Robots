function FWP = get_FWP(Grid)

Ngrid = length(Grid);
FWP = cell(Ngrid,1);
for i_g = 1:Ngrid
    gg = Grid{i_g};
    J1 = getJacobian(gg(1,:));
    J2 = getJacobian(gg(2,:));
    J3 = getJacobian(gg(3,:)); 

    fwp{1} = get_FWP_oneLeg(J1);
    fwp{2} = get_FWP_oneLeg(J2);
    fwp{3} = get_FWP_oneLeg(J3);

    A = [fwp{1}.A;fwp{2}.A;fwp{3}.A];
    b = [fwp{1}.b;fwp{2}.b;fwp{3}.b];

    FWP{i_g} = Polyhedron(A,b);
    FWP{i_g}.minHRep;
    FWP{i_g}.minVRep;
end

end

function FWP = get_FWP_oneLeg(J)

p = getParams;
umax = p.umax;
mu = p.mu;

A = [[ 1 0;...
         -1 0;...
          0 1;...
          0 -1]*J';...
          1 -mu;...
         -1 -mu];
     
b = [umax * [1;1;1;1];...
         0;...
         0];

FWP = Polyhedron(A,b);
FWP.minHRep;
FWP.minVRep;

end

function [J,q] = getJacobian(pt)
p = getParams;
l1 = p.l1;
l2 = p.l2;

th = atan2(pt(2),pt(1));
r = norm(pt,2);
cos_alpha = (l1^2+r^2-l2^2)/(2*l1*r);
alpha = acos(cos_alpha);

cos_beta = (l1^2+l2^2-r^2)/(2*l1*l2);
beta = acos(cos_beta);

q1 = -(pi - th + alpha);
q2 = pi - beta;
q = [q1;q2];

J = fcn_J([q1,q2],[l1,l2]);

end

