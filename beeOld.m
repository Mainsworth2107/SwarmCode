%% Function that performs one point swapping on potential solutions

function sol = beeOld(A,i,neighbour)
    test = A(i,:); %Test variable to snsure swapping is being perfomred correctly
    global allObjs;
    objs = height(allObjs);
    
    %Selects a random element of the solution to be altered
    idxs = floor(rand(1,1)*length(A(i,:)))+1;
    
    %Method for changing two variables at random
%     cut = 0;
%     while(idxs(1) == idxs(2))
%        idxs(2) = floor(rand*length(A(i,:)))+1;
%        cut = cut + 1;
%        %Statement for saftey only
%        if(cut > 100)
%            break;
%        end
%     end
    
    % Determines the maginitude change of the variable by looking at the neighbouring
    % solution as in continous ABC.
   
    %Unlike cintinous ABC, the only useful information obtainable for this
    %sepcific cas is if the values are differnt, and their numberical
    %differnce has no bearing when swapping.
    
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