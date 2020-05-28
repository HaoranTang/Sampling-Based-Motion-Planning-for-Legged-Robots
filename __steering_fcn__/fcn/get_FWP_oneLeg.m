function [AWP,FWP] = get_FWP_oneLeg(J,p)

umax = p.umax;
mu = p.mu;


A_AWP = [ 1 0;...
         -1 0;...
          0 1;...
          0 -1]*J';
      
b_AWP = umax * [1;1;1;1];

AWP = Polyhedron(A_AWP,b_AWP);


A_FWP = [A_AWP;...
          1 -mu;...
         -1 -mu];
     
b_FWP = [b_AWP;...
         0;...
         0];

FWP = Polyhedron(A_FWP,b_FWP);






