%% Function that returns the fitness of a given allocation

function X = beeFit(FoodNumber,A,robots,qualities,mode)
    %Fitness calclation (for each robot)
    global allObjs
    objs = height(allObjs);
    X = zeros(1,FoodNumber);
    
    for i = 1:FoodNumber
        %Count and dist are used to find mae in loop
        
        counts = zeros(1,objs); %Allocation counts for each object 
        dist = 0; %Total distance travelled by the robots
        
        % Count allocations for each robot (exculding pre allocated ones)
        for k = 1:objs
            counts(k) = length(find( A(i,:)==k ));
        end
        
        %Adding static robots to counts and normalising
        counts = counts + 1; 
        counts = counts ./ (sum(counts));
        
        %Calculate the total distance travelled by all robots. (exculding
        %pre allocated ones)
        for k = 1:length(A(i,:)) %First objs robots ignored
            dist = dist + distEu(robots{k+objs}.pose(1:2), allObjs(A(i,k),1:2) );
        end

        %Averages the distance across all robots
        dist = dist / length(robots);
        
        %Finds the MAE for the input allocation
        Error = (1/objs) * sum(abs(qualities - counts));
        
        %As defined in a follow up paper: fitness = 1/(mae*dist)
        % [Distributed Bees Algorithm Parameters Optimization for 
        % a Cost Efficient Target Allocation in Swarms of Robots]
        
        %To avoid division by zero, the MAE is given a statistically
        %insignificant bais, currently set to numRobots/1000.

        %mae is of greater importance than distance because it has a smaller
        %(larger inverse) order of magnitude. 
        
        switch(mode)
            case 0 %Case 0 is the standard fitness calculation
                X(i) = (1/  ((Error + (0.001/length(robots)) ) * dist ) );
%                     X(i) = (1/  ((Error +1) * dist ) );
            
            case 1 % Cases 1 and 2 return error and distance repscitvlry 
                   % for testing and output purpoouses
                X(i) = Error;
            case 2
                X(i) = dist;
        end
    end
    
end