function F = cst_PiecewiseMccomick(x_in,M,x_interval,y_interval)
% x = linspace(x_interval(1),x_interval(2),21);
% y = linspace(y_interval(1),y_interval(2),21);
% [X,Y] = meshgrid(x, y);
% w = X.*Y;
% surf(X,Y,w);

%Piecewise McCormick Bilinear Estimation
x_min = x_interval(1);
x_max = x_interval(2);

y_min = y_interval(1);
y_max = y_interval(2);

bin = binvar(M,M);
x_grid = linspace(x_min,x_max,M+1);
y_grid = linspace(y_min,y_max,M+1);

H = zeros(6,3,M);
k = zeros(6,M);

bigM = 100;

F = [];
F = [F,sum(sum(bin)) == 1];

%% McCormick Envelope
for ii = 1:M
    for jj = 1:M
        xiL = x_grid(ii);
        xiU = x_grid(ii+1);
        yiL = y_grid(jj);
        yiU = y_grid(jj+1);
        H(1,1:3,ii,jj) = [yiL xiL -1];
        k(1,ii,jj) = yiL*xiL;
        H(2,1:3,ii,jj) = [yiU xiU -1];
        k(2,ii,jj) = yiU*xiU;
        H(3,1:3,ii,jj) = [-yiU -xiL 1];
        k(3,ii,jj) = -xiL*yiU;
        H(4,1:3,ii,jj) = [-yiL -xiU 1];
        k(4,ii,jj) = -yiL*xiU;
        H(5,1:3,ii,jj) = [1 0 0];
        k(5,ii,jj) = xiU;
        H(6,1:3,ii,jj) = [-1 0 0];
        k(6,ii,jj) = -xiL;
        
        F = [F, H(:,:,ii,jj)*x_in - k(:,ii,jj) - bigM * (1 - bin(ii,jj)) <= 0];
        
%         plot(H(:,:,ii,jj)*x_in <= k(:,ii,jj));hold on
    end
end




