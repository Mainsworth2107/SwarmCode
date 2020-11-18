%% Function that manages parameter changes for potential solutions
function [sol] = shiftSol(A,FoodNumber,i)

    %/*A randomly chosen solution is used in producing a mutant solution of the solution i*/
    
    %Neighbour represnts another whole solution (food source)
    rSol = fix(rand*(FoodNumber))+1;
    neighbour=fix(rand*(FoodNumber))+1;
    
    method = i;
    
    %Randomly selected solution must be different from the solution i        
    while(neighbour==method)
        neighbour=fix(rand*(FoodNumber))+1;
    end

    %Alters the input solution according to single point swapping
%     sol = beeNew(A,i,neighbour);    
    sol = beeShift(A,method,neighbour);    
end