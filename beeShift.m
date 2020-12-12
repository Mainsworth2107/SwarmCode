%% Function that performs one point swapping on potential solutions
 
function sol = beeShift(A,i,neighbour)
    test = A(i,:); %Test variable to ensure swapping is being performed correctly
    global allObjs;
    objs = height(allObjs);
    
    %Selects a random element of the solution to be altered
    idxs = floor(rand(1,1)*length(A(i,:)))+1;
    
    
    % Determines the magnitude change of the variable by looking at the neighbouring
    % solution as in continuous ABC.
   
    %Unlike continuous ABC, the only useful information obtainable for this
    %specific case is if the values are different, and their numerical
    %difference has no bearing when swapping.
    
    %As such a swap will occur any time the variables differ.
    %(this will lead to convergence)
    for j=1:1
        if ~(A(i,idxs(j)) == A(neighbour,idxs(j)))
            A(i,idxs(j)) = mod( A(i,idxs(j)) + ceil(rand*(objs-1)) ,objs);
            if A(i,idxs(j)) == 0
                A(i,idxs(j)) = objs;
            end
        end
    end
    sol = A(i,:);
end