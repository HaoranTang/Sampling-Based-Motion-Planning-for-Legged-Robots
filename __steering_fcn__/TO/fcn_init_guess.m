function guess = fcn_init_guess(p)

time = [0 p.Tst];
vin = p.vin;
vout = p.vout;
nGrid = p.nGrid;
Tst = p.Tst;
g = p.g;

xf = vout(1) * Tst/2;
zf = vout(2) * (Tst/2) - 1/2 * g * (Tst/2)^2;
vxf = vout(1);
vzf = vout(2) - g * (Tst/2);

xi = vin(1) * (-Tst/2);
zi = vin(2) * (-Tst/2) - 1/2 * g * (-Tst/2)^2;
vxi = vin(1);
vzi = vin(2) - g * (-Tst/2);

qi = IK([xi;zi],p);
qf = IK([xf;zf],p);
dqi = [vxi;vzi];
dqf = [vxf;vzf];
Xi = [qi;dqi];
Xf = [qf;dqf];

tSpan = time([1,end]);
guess.time = linspace(tSpan(1),tSpan(2),nGrid);

guess.Xi = [xi;zi;vxi;vzi];
guess.Xf = [xf;zf;vxf;vzf];

guess.co = [[p.mass*p.g*ones(1,p.nco)];...
            [p.mass*p.g*ones(1,p.nco)]];

guess.state = interp1(tSpan',[Xi,Xf]',guess.time')';

Uguess = [0;0];
guess.control = interp1(tSpan',[Uguess,Uguess]',guess.time')';


end