function bool_inWS = fcn_ifInWS(X,p)

L = p.L;
l1 = p.l1;
l2 = p.l2;

x = X(1);
z = X(2);
th = X(3);
pf_frt = X(4:5);
pf_bck = X(6:7);


% --- leg kinematics ---
p_COM = [x;z];                                  % COM
p_hip_frt = p_COM + L/2 * [cos(th);sin(th)];    % front hip
p_hip_bck = p_COM - L/2 * [cos(th);sin(th)];    % back hip

% vector from hip to foot
r_frt = pf_frt - p_hip_frt;
r_bck = pf_bck - p_hip_bck;

% --- conditions for workspace ---
% r_min <= |r| <= r_max for both front and back
% p_hip above ground
% p_knee above ground
% (future)
% |r_bck| > r_max and r_frt bounded (front stance)
% |r_frt| > r_max and r_bck bounded (back stance)

r_min = 0.05;
r_max = 0.26;
% bool var indicating if leg is in stance
bool_frt_bdd = (r_min <= norm(r_frt)) && (norm(r_frt) <= r_max);
bool_bck_bdd = (r_min <= norm(r_bck)) && (norm(r_bck) <= r_max);

if bool_frt_bdd && bool_bck_bdd     % double stance
    [J_bck,q_bck] = getJacobian(-r_bck);
    [J_frt,q_frt] = getJacobian(-r_frt);
    
    p_knee_bck = p_hip_bck + l1 * [cos(q_bck(1));sin(q_bck(1))];
    p_knee_frt = p_hip_frt + l1 * [cos(q_frt(1));sin(q_frt(1))];
    
    bool_knee_above_ground = (p_knee_bck(2) > 0.01) && (p_knee_frt(2) > 0.01);
    bool_hip_above_ground = (p_hip_bck(2) > 0.01) && (p_hip_frt(2) > 0.01);
    
    if bool_knee_above_ground && bool_hip_above_ground % knee above ground
        bool_inWS = 1;
    else
        bool_inWS = 0;
    end
else
    bool_inWS = 0;
end
    


end