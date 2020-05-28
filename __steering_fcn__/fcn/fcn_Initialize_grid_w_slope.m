function [grid0] = fcn_Initialize_grid_w_slope(Nr,Nth,slope)

rSpan = linspace(0.26,0.08,Nr);
thMin = -pi*0.3 + pi/2;
thMax = pi*0.3 + pi/2;
r_mat = repmat(rSpan',1,Nth);
th_mat = get_th_mat(rSpan,Nth,thMax,thMin);

X_mat = r_mat.*cos(th_mat);
Z_mat = r_mat.*sin(th_mat);

grid0 = cell(Nr-1,2*(Nth-1));

for i_r = 1:Nr-1
    for i_th = 1:Nth-1
        grid0{i_r,2*i_th-1} = [X_mat(i_r,i_th),Z_mat(i_r,i_th);
                                X_mat(i_r,i_th+1),Z_mat(i_r,i_th+1);
                                X_mat(i_r+1,i_th),Z_mat(i_r+1,i_th)];
        grid0{i_r,2*i_th-1} = (rot2D(slope) * (grid0{i_r,2*i_th-1})')';
                            
        grid0{i_r,2*i_th  } = [X_mat(i_r+1,i_th),Z_mat(i_r+1,i_th);
                                X_mat(i_r+1,i_th+1),Z_mat(i_r+1,i_th+1);
                                X_mat(i_r,i_th+1),Z_mat(i_r,i_th+1)];
        grid0{i_r,2*i_th  } = (rot2D(slope) * (grid0{i_r,2*i_th  })')';
    end
end

grid0 = grid0';
% --- the grid ---
grid0 = grid0(:);
% ----------------

end



