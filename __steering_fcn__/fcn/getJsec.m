function [JJT] = getJsec(grid)

N = length(grid);
JJT = cell(N,1);

for i_N = 1:N
    g = grid{i_N};
    J1 = getJacobian(g(1,:));
    J2 = getJacobian(g(2,:));
    J3 = getJacobian(g(3,:));
    
    %% force
    A1 = J1 * J1';
    A2 = J2 * J2';
    A3 = J3 * J3';
    
    A = sdpvar(2);
    
    F = [];
    F = [F,A >= 0];
    F = [F,A >= A1];
    F = [F,A >= A2];
    F = [F,A >= A3];
    
    obj = trace(A);
    ops = sdpsettings('verbose',0,'solver','mosek');
%     ops = [];
    sol = optimize(F,obj,ops);
    disp(sol.info)
    Aval = value(A);

    JJT{i_N} = Aval;
    
    fprintf('grid %d/%d is complete!\n',i_N,N);
    
end

end








