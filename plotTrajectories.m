function plotTrajectories(pars, Ndim, S, G, O, Trajectories, animateFlag, CBFflag, APFflag)

    %%%%%%%%%%%%%%%%%%%%%%GRAPHICS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Set LateX as default interpreter
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
    set(groot, 'defaultLegendInterpreter','latex');
    set(groot, 'defaultTextInterpreter','latex')
    %Standard size for labels and title
    labelsize = 20;
    titlesize = 30;
    
    %If the chosen alphas are the same of the article, choose their same color
    %palette, otherwise choose colors from the HSV color palette.
    if(CBFflag && ~APFflag)
        par = "$\alpha$";
        if(pars(1) == 0.5 && pars(2) == 1 && pars(3) == 2 && pars(4) == 100)
            colors = ['blue', 'red', "#EDB120", "#7E2F8E"];
        else
            colors = hsv(numel(alpha));
        end
        titlestring = "Obstacle avoidance using CBF";
    elseif(~CBFflag && APFflag)
        par = "$\rho_0$";
        %If the chosen rho0 are the same of the article, choose their same color
        %palette, otherwise choose colors from the HSV color palette.
        if(pars(1) == 0.1 && pars(2) == 0.25 && pars(3) == 0.5 && pars(4) == 1)
            colors = ["#7E2F8E", "#EDB120", 'red', 'blue' ];
        else
            colors = hsv(numel(rho0));
        end
        titlestring = "Obstacle avoidance using APF";
    end
    
    %Plot results
    figure(1);clf;
    hAxes = axes;
    hold on
    axis equal
    
    %Plot start and goal
    if(Ndim == 2)
        plot(S(1), S(2),"or",MarkerFaceColor="red")
        plot(G(1), G(2),"og",MarkerFaceColor="green")
    elseif(Ndim==3)
        plot3(S(1), S(2),S(3),"or",MarkerFaceColor="red")
        plot3(G(1), G(2),G(3),"og",MarkerFaceColor="green")
    end
    
    %Plot obstacles
    if(Ndim == 2)
        drawcircles(hAxes,O.C(:,1:Ndim),O.R);
    elseif(Ndim == 3)
        for i = 1:numel(O.R)
            [X,Y,Z] = sphere;
            X2 = X * O.R(i);
            Y2 = Y * O.R(i);
            Z2 = Z * O.R(i);
            surf(X2 + O.C(i,1),Y2 + O.C(i,2),Z2 + O.C(i,3),FaceColor=[0.2,0.2,0.2],FaceAlpha=0.3)
        end
    end
    
    
    %Add title, labels and limits
    
    title(titlestring, FontSize=titlesize)
    xlabel("$x$ position [m]","FontSize", labelsize)
    ylabel("$y$ position [m]","FontSize", labelsize)
    if(Ndim == 3)
        zlabel("$z$ position [m]","FontSize", labelsize)
        view([37.5, 30])
        grid on
    end
    
    [axisLimits, deltas] = findAxisLimits(Trajectories,O,Ndim);
    
    xlim([axisLimits(1)-deltas(1),axisLimits(2)+deltas(1)])
    ylim([axisLimits(3)-deltas(2),axisLimits(4)+deltas(2)])
    if(Ndim == 3)
        zlim([axisLimits(5)-deltas(3),axisLimits(6)+deltas(3)])
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%PLOT TRAJECTORIES%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(animateFlag)
        MaxTime = 0; 
        for k = 1:numel(pars)
            MaxTime = max(MaxTime, size(Trajectories{k},1));
        end
        Divider = 10;
        for t = 2:floor(MaxTime/Divider)
            for k = 1:numel(pars)
                if(t<=size(Trajectories{k},1))
                    if(Ndim == 2)
                        plot([Trajectories{k}(t-1,1),Trajectories{k}(t,1)], [Trajectories{k}(t-1,2),Trajectories{k}(t,2)], "-", color=colors(k), LineWidth=3)
                    elseif(Ndim == 3)
                        plot3([Trajectories{k}(t-1,1),Trajectories{k}(t,1)], [Trajectories{k}(t-1,2),Trajectories{k}(t,2)],[Trajectories{k}(t-1,3),Trajectories{k}(t,3)], "-", color=colors(k), LineWidth=3)
                    end
                end
                refresh
                drawnow limitrate
            end
        end
        %The simulation gets increasingly slower the closer we get to the goal
        %point, because the control (the speed) is proportional to the
        %distance. For this reason, past some fraction of the maximum time, we
        %start plotting 100 points at once instead of just 1.
        for t = floor(MaxTime/Divider)+1:100:MaxTime-100
            for k = 1:numel(pars)
                if(t+100<=size(Trajectories{k},1))
                    if(Ndim == 2)
                        plot(Trajectories{k}(t:t+100,1), Trajectories{k}(t:t+100,2), "-", color=colors(k), LineWidth=3)
                    elseif(Ndim == 3)
                        plot3(Trajectories{k}(t:t+100,1), Trajectories{k}(t:t+100,2), Trajectories{k}(t:t+100,3), "-", color=colors(k), LineWidth=3)
                    end
                end
                refresh
                drawnow limitrate
            end
        end
    else
        for k = 1:numel(pars)
            if(Ndim == 2)
                plot(Trajectories{k}(:,1), Trajectories{k}(:,2), "-", color=colors(k), LineWidth=3)
            elseif(Ndim==3)
                plot3(Trajectories{k}(:,1), Trajectories{k}(:,2),Trajectories{k}(:,3), "-", color=colors(k), LineWidth=3)
                view([37.5, 30])
                grid on
            end
        end
    end
    %%%%%%%%%%%%%%%%%%%END PLOT TRAJECTORIES%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%BUILD LEGEND%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    legendLabel = strings(1,numel(O.R)+numel(pars)+2);
    legendLabel(1:2) = ["Start", "Goal"];
    legendLabel(3:2+numel(O.R)) = ["Obstacle", repmat("", 1, numel(O.R)-1)];
    legendLabel(end-numel(pars)+1:end) = strcat(par," = ",string(pars));
    legend(legendLabel,"Location","northwest")
    %%%%%%%%%%%%%%%%%%%%%%%%END BUILD LEGEND%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end