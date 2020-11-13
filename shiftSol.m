%% Function that mananges parameter changes for potential solutions

function [sol] = shiftSol(A,FoodNumber,i)
    %rSol = fix(rand*(FoodNumber))+1;
    %Represents potential solution 
    Method = i;
    
    %Neighbour represnts another solution(food source) within the space.
    neighbour=fix(rand*(FoodNumber))+1;    

    %Randomly selected neighbor solution must be different from the solution i*/        
    while(neighbour==Method)
        neighbour=fix(rand*(FoodNumber))+1;
    end

    %Alters the input solution according to single point swapping
%     sol = beeNew(A,i,neighbour);    
    sol = beeOld(A,Method,neighbour);    
end