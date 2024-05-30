function [axisLimits, deltas] = findAxisLimits(Trajectories,O,Ndim)
    % Determine the axis limits on the basis of the obstacles edges and the
    %coordinates of the trajectories    
    axisLimits = zeros(1,Ndim * 2);
    deltas = zeros(1,Ndim);
    
    for i = 1:numel(axisLimits)
        axisLimits(i) = (-1)^(i-1) * inf;
    end
    for i = 1:numel(Trajectories)
        for j = 1:Ndim
            axisLimits(2*j-1) = min(axisLimits(2*j-1), min(Trajectories{i}(:,j)));
            axisLimits(2*j) = max(axisLimits(2*j), max(Trajectories{i}(:,j)));
            
        end
    end
    for i = 1:numel(O.R)
        for j = 1:Ndim
            axisLimits(2*j-1) = min(axisLimits(2*j-1), O.C(i,j)-O.R(i));
            axisLimits(2*j) = max(axisLimits(2*j), O.C(i,j)+O.R(i));
            
        end
    end
    for j = 1:Ndim
        deltas(j) = (axisLimits(2*j)-axisLimits(2*j-1))/10;
    end
end