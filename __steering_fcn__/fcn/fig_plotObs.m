function terrain = fig_plotObs(infObs)
    
    % infObs: [pObs;...
    %          slope]
    N = size(infObs,2)-1;
    if N == 1
        plot(infObs(1,:),infObs(2,:),'k','linewidth',2)
    else
        terrain = zeros(2,2*N);
        for ii = 1:N
            terrain(:,2*ii-1) = infObs(1:2,ii);
            terrain(:,2*ii) = ...
               [infObs(1,ii+1);...
                infObs(2,ii)+tan(infObs(3,ii))*(infObs(1,ii+1)-infObs(1,ii))];
        end
        plot(terrain(1,:),terrain(2,:),'k','linewidth',2);
        axis equal
    end
end