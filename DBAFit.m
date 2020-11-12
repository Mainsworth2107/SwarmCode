%% Function that calculates the allocation of a robot.
 
function [X,Y] = DBAFit(poseIn)
    % Global knowledge of object positions
    global allObjs
    
    %Array that stores the distance of each robot from each object.
    costs = zeros(1,height(allObjs));
    
    %Control Parameters
    a = 1;b = 1;
    
    vis = costs; %Visibility measures (inverse of distance)
    qual = costs; %Normalised qualities of objects
    total = 0; %Sum of product of biased quantities and visibilities
    p = costs; %Calculated probabilities
    
    % Calculates the total quality for the normalisation step
    totQual = 0;
    for i = 1:length(costs)
        totQual = totQual + allObjs(i,4);
    end
    
    % For each object:
    for i = 1:length(costs
        
        % Finds the Euclidian distance of each object from the robot
        costs(i) = distEu(poseIn(1:2),allObjs(i,1:2)); 
        
        %Calculates visibility
        vis(i) = 1/costs(i);
        
        % Calculates normalised quality values
        qual(i) = allObjs(i,4) / totQual;
        
        % Adds to the sum of product of biased quantities and visibilities
        total = total + ( (qual(i)^a) * (vis(i) ^ b)   );
    end
    
    for i=1:length(costs)
        %Finds the probability values for each object.
        p(i) = ((qual(i)^a) * (vis(i) ^ b)) / total;
    end
    
    % Uses roulette wheel selection to determine allocation
    choice = rand;
    total = 0;
    
    for i=1:length(p)
        total = total + p(i);
        if(total > choice)
            A = i;
            break
        end
    end
    X = A;
    Y = sum(costs);
end