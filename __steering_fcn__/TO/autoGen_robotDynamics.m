function [ddx,ddy,ddth] = autoGen_robotDynamics(x,y,u1x,u1y,u2x,u2y,m,J,g,L)
%AUTOGEN_ROBOTDYNAMICS
%    [DDX,DDY,DDTH] = AUTOGEN_ROBOTDYNAMICS(X,Y,U1X,U1Y,U2X,U2Y,M,J,G,L)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    07-May-2020 12:43:21

t2 = 1.0./m;
ddx = t2.*(u1x+u2x);
if nargout > 1
    ddy = t2.*(u1y+u2y-g.*m);
end
if nargout > 2
    ddth = (L.*u1y.*(-1.0./2.0)+L.*u2y.*(1.0./2.0)-u1y.*x-u2y.*x+u1x.*y+u2x.*y)./J;
end
