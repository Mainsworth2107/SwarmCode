function [fitness,trial] = updateSol(sol,fitness,trial,robots,qualities)
    
    FitnessSol = beeFit(1,sol,robots,qualities);

    % /*a greedy selection is applied between the current solution i and its mutant*/
    if (FitnessSol>fitness) %/*If the mutant solution is better than the current solution i, replace the solution with the mutant and reset the trial counter of solution i*/
        fitness=FitnessSol;
        trial=0;
    else
        trial=trial+1; %/*if the solution i can not be improved, increase its trial counter*/
    end
    
end