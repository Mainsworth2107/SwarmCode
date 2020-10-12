%Assume that all group visible objects are known

%Need to have a way of obtaining known objs and robots before.
function taskAllo(knownObjs)
    %% Initiation
    %Receives known robots
    global others;
    knownBots = others;
    %AP best seems to be mergeable with Pbest
    A = rand(1,(length(knownBots)))*length(knownObjs) + 1; %Allocation is real but interpreted as natural
    AGbest = floor(A);
    its = 0;
    Pbest(1) = floor(A);
    recd = [];  
    limit = 500;
    % Fitness is EU distance between a robot and its current allocation
    while(its < 500)
        %% Personal best update (also handels messaging)
        
        %Updates PB if A has got a new PB
        if getFit(floor(A),knownObjs,knownBots) > getFit(Pbest(1),knownObjs,knownBots)
            Pbest(1) = getFit(floor(A));
        end
        %% Comms, re do
        robots{knownBots(1)}.msg = -1;
        
        for i= 2:len(knownBots)  
            idx = knownBots(i);
            robots{idx}.msg = [robots{idx}.msg;Pbest];
        end
        got = 1;
        its = 1;
        while(got < length(knownBots))
            tmp = size(robots{knownBots(1)}.msg);
            got = tmp(1);
            if(its >= limit)
                disp('Error with comms');
                break;
            end
        end
        
        %% 
        recd = robots{knownBots(1)}.msg;
        tmp = size(recd);
        % Here recd is storing all received allocations (tmp protects
        % aganist no comms)
        if(tmp > 1)
            for i = 2:tmp
                %evaluate other solutions
                Pbest(i) = getFit(recd(1,:),knownObjs,knownBots); %Fitness of robot i's APbst
            end
        end
        
        %% Global Best update
        idBest = 1;
        min = Pbest(1);
        for i =2:numRobots
            if Pbest(i) < min
                idBest = min;
                min = Pbest(i);
            end
        end
        
        %% Global best diffusion
        %Diffuses out global best
        if idBest == robots{knownknownBots(1)}.id
            AGbest = min;
            for i= 2:len(knownBots)  
                idx = knownBots(i);
                robots{idx}.msg = [robots{idx}.msg;Pbest];
            end
        else
            %Waits to recive best allocation form a friend
            robots{knownBots(1)}.msg = [];
            its = 1;
            while (isempty(robots{knownBots(1)}.msg))
                if(its >= limit)
                    disp('Error with comms');
                    break;
                end
            end
            if( ~isempty(robots{knownBots(1)}.msg))
                AGbest = robots{knownBots(1)}.msg;
            end
        end
        
        %% Move the particle 
        V = 0; % FOR TESTING, REMOVE
        % Calculating V (need previous velocity)
        c1 = 2; c2 = c1;
        V = (w*V) + (c1*rand*Pbest(1)) + (c2*rand*AGbest);
        %Updating position
        A = A + V;
    end
end

% Calculates the fitness for a given allocation

function X = getFit(input,knownObjs,knownBots) %input is allocation array (1xN array)
    sum = 0; %Calculation variable
    currentObj = 0; %Currently considered object ID
    %We need to have a set of known objects (knownObjs)
    %e.g (x,y,2;x,y,6 ...)
    
    %and a set of known robots (ID can be prioirty for this alg)
    %e.g (2,3,4)
    
    for i = 1:length(input)
        % Discovers information about object X
        for j = 1:length(knownObjs)
            if knownObjs(j,3) == input(i)
                currentObj = knownObjs(j,:);
                break
            end
        end
        sum = sum + distEu(currentObj(1:2),robots{knownBots(i)}.pose(1:2));
    end
    X = sum;
    return
end