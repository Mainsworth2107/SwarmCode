function [sol] = shiftSol(A,FoodNumber,i)

    %/*A randomly chosen solution is used in producing a mutant solution of the solution i*/
    
    %Neighbour represnts another whole solution (food source)
    rSol = fix(rand*(FoodNumber))+1;
    neighbour=fix(rand*(FoodNumber))+1;

    %/*Randomly selected solution must be different from the solution i*/        
    while(neighbour==rSol)
        neighbour=fix(rand*(FoodNumber))+1;
    end

   %Assumption that bounds are between -x and x ((rand-0.5)*2 statement)
   %  /*v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) */
   
   % Check this
%    sol = beeSwap(A,i,Param2Change,neighbour);
    sol = beeNew(A,rSol,neighbour);    

end