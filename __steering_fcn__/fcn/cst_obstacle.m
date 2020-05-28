function F = cst_obstacle(infObs,istep,b_Obs,xTD,zTD,bigM,Fx_co,Fz_co,p)

e = p.eMargin;
mu = p.mu;

Nobs = size(infObs,2) - 1;
pObs = infObs(1:2,:);
slope = infObs(3,:);

s_check = linspace(0,1,10);
Fx_check = polyval_bz(Fx_co(istep,:),s_check);
Fz_check = polyval_bz(Fz_co(istep,:),s_check);

F = [];
F = [F, (sum(b_Obs(:,istep)) == 1):'sum(b_Obs)==1'];

if Nobs == 1
    F = [F,zTD(istep+1) == pObs(2,1)];
else
    for i_obs = 1:Nobs
        % the touchdown point stays on the segmented terrain
        F = [F, (pObs(1,i_obs) + e - bigM*(1-b_Obs(i_obs,istep)) <= ...
            xTD(istep+1) <= pObs(1,i_obs+1) - e + bigM*(1-b_Obs(i_obs,istep))):'xTD constraint'];
        F = [F, (pObs(2,i_obs)+tan(slope(i_obs))*(xTD(istep+1)-pObs(1,i_obs))-bigM*(1-b_Obs(i_obs,istep)) <= ...
            zTD(istep+1) <=  pObs(2,i_obs)+tan(slope(i_obs))*(xTD(istep+1)-pObs(1,i_obs))+bigM*(1-b_Obs(i_obs,istep))):'zTD constraint'];
        
        % check GRF
        F = [F, ([1 -mu; -1 -mu] * rot2D(slope(i_obs)) * [Fx_check;Fz_check] <= bigM*(1-b_Obs(i_obs,istep))):'friction cone'];
        F = [F, (-bigM*(1-b_Obs(i_obs,istep)) <= [0 1] * rot2D(slope(i_obs)) * [Fx_check;Fz_check]):'positive normal force']; 
        
    end
end

% b_Obs = [Nobs,Nstep]
% F = [F, b_Obs(1,1) == 1];


