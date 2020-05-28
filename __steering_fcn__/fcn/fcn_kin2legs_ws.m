function [X_ws] = fcn_kin2legs_ws(p)
% function [JL,JR,chain,qL,qR] = fcn_kin2legs_ws(X,p)
% 

% parameters
L = p.L;
l1 = p.l1;
pf_bck = p.pf_bck;
pf_frt = p.pf_frt;
xmin = -L;        xmax = L;         Nx = 20;
zmin = 0;           zmax = 0.26;        Nz = 20;
thmin = -1;         thmax = 1;          Nth = 20;



%%
x_vec = linspace(xmin,xmax,Nx);
z_vec = linspace(zmin,zmax,Nz);
th_vec = linspace(thmin,thmax,Nth);
[X_mat,Z_mat,TH_mat] = meshgrid(x_vec,z_vec,th_vec);

X_ws = zeros(0,7);
idx = 1;
for ix = 1:Nx
    for iz = 1:Nz
        for ith = 1:Nth
            % decompose X
            x = X_mat(ix,iz,ith);
            z = Z_mat(ix,iz,ith);
            th = TH_mat(ix,iz,ith);
            X = [x;z;th;pf_frt;pf_bck];
            bool_inWS = fcn_ifInWS(X,p);
            
            if bool_inWS
                X_ws(idx,:) = X';
                idx = idx + 1;
            end
        end
    end
end


% %% reconstrust joint positions
% % center of mass
% p_COM = [x_vec;z_vec];
% 
% % hip joint position
% p_hipR = p_COM + L/2 * [cos(th_vec);sin(th_vec)];
% p_hipL = p_COM - L/2 * [cos(th_vec);sin(th_vec)];
% 
% % leg Jacobian
% [JL,qL] = getJacobian(p_hipL - p_fL);
% [JR,qR] = getJacobian(p_hipR - p_fR);
% 
% % knee joint position
% p_kneeR = p_hipR + l1*[cos(qR(1));sin(qR(1))];
% p_kneeL = p_hipL + l1*[cos(qL(1));sin(qL(1))];
% 
% % chain of joints for plottting
% chain = [p_fL,p_kneeL,p_hipL,p_hipR,p_kneeR,p_fR];
% 
% 
% X_ws = 0;
