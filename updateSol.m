%% Refernces
%All refernces given in main.m
%
%% Function that handles comparing a mutant and original solution.

function [fitness,trial] = updateSol(sol,fitness,trial,robots,qualities)
    
    FitnessSol = beeFit(1,sol,robots,qualities,0);

    % A greedy selection is applied between the current solution i and its mutant
    % If the mutant solution is better than the current solution i, 
    % replace the solution with the mutant
    
    if (FitnessSol>fitness) 
        fitness=FitnessSol; %Updates the fitness values saved for the solution
        trial=0;            %Resets the trial counter saved for the solution
    else
        trial=trial+1; %If the solution can not be improved, increase its trial counter
    end
    
end