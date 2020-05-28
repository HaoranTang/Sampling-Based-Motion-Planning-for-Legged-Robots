function [J] = fcn_J(q,params)


  J(1,1)=- params(2)*sin(q(1) + q(2)) - params(1)*sin(q(1));
  J(1,2)=-params(2)*sin(q(1) + q(2));
  J(2,1)=params(2)*cos(q(1) + q(2)) + params(1)*cos(q(1));
  J(2,2)=params(2)*cos(q(1) + q(2));
  
% GRF is the reaction force of the force that the joint torque generate
J = -J;

 