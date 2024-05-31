function Path = computeTrajectoryAPF(S, G, O, rho0, dT, Katt, Krep, MaxTime, Ndim)
% This function takes as inputs:
% 1. S: the starting point (a 2-dimensional row vector)
% 2. G: the goal point
% 3. O: a struct made of two fields: C (centers) and R (radii), to be
% provided as individual row vectors and scalars, respectively;
% 4. v0: the initial velocity. If it is not set (provided as []) the
% function will use v0 = -Katt * (S - G)';
% 5. alpha: the constant factor of the CBF constraint:
% grad(h)^T . v >= -alpha * h
% where ^T indicates transposition; 
% 6. dT: the timestep (in seconds); 0.001s is chosen, corresponding to a
% refresh rate of 1kHz;
% 7. Katt: the attraction coefficient from the start to the goal;
% 8. MaxTime: the maximum time for the simulation to converge (i.e.
% determine a path from S to G). For the S and G in the paper, MaxTime =
% 10000 is sufficient.
% 9. Ndim: selects the number of dimensions: if Ndim = 2 is selected, any
% 3D point gets its third component trimmed.
%
% The function provides as output a list of points which correspond to the
% trajectory determined by the algorithm. Each point is a row vector.
    
    %Default parameters
    if(nargin<1)
        S = [0,0];
        G = [3,5];
        O.C(1,:) = [1,2];
        O.R(1) = 0.5;
        O.C(2,:) = [2.5,3];
        O.R(2) = 0.5;
        dT = 0.001;
        Katt = 1;
        MaxTime = 10000;
        Ndim = 2;
    end
    
    %Set a precision level beyond which the goal is considered reached. It
    %is defined as 0.2% of the Euclidean distance between S and G:
    Precision = norm(S-G) * 0.002;

    %Sets the initial position as the starting point
    Path(1,:) = S;

    %Resets timer
    t = 1;

    %Sets a flag that stops the computation once G is reached within
    %precision or if time is over
    continueFlag = true;
    timeupFlag = false;
    
    prog = 0;
    fprintf(1,'Computation Progress: %3d%%\n',prog);
    rho = zeros(1,numel(O.R));
    v = zeros(MaxTime,Ndim);
    
    while(continueFlag && ~timeupFlag)
        gradUatt = Katt * (Path(t,:) - G);
    
        gradUrep = 0;
        
        for i = 1:numel(O.R)
            rho(i) = norm(Path(t,:) - O.C(i,1:Ndim)) - O.R(i);
            if(rho(i) <= rho0)
                gradUrep = gradUrep + Krep * (1/rho(i) - 1/rho0) / rho(i) ^ 3 * (O.C(i,1:Ndim) - Path(t,1:Ndim));
            end
        end
        v(t,:) = - gradUatt - gradUrep;
    
        
        dR = dT * v(t,:);
        t = t + 1;
        Path(t,:) = Path(t-1,:) + dR;
        
        if(mod(100*(t/MaxTime),1)==0)
            prog = ( 100*(t/MaxTime) );
	        fprintf(1,'\b\b\b\b%3.0f%%',prog); pause(0.1); % Deleting 4 characters (The three digits and the % symbol)
        end
        


        if(norm(Path(t,:) - G) < Precision)
            %If we are close enough to the goal, the exit the loop
            continueFlag = false;
        end
        
        if(t>MaxTime)
            timeupFlag = true;
        end
    end
    fprintf(1,'\b\b\b\b%3.0f%%',100); pause(0.1); % Deleting 4 characters (The three digits and the % symbol)
    fprintf('\n'); % To go to a new line after reaching 100% progress
    if(timeupFlag && continueFlag)
        fprintf(1,strcat("Simulation did not converge after ", num2str(MaxTime), " steps\n"))
    elseif(~timeupFlag && ~continueFlag)
        fprintf(1,strcat("Simulation for rho0 = ", num2str(rho0)," successfully converged within precision\n"))
    end
    
end