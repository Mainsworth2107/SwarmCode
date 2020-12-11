%% RDABC Solution Shifting
 
function sol = beeNew(A,i,neighbour)
    test = A(i,:);
    
    % Getting global object positions
    global allObjs;
    objs = height(allObjs);
    
    % Shorthand for current and neighbouring solution
    P = A(i,:);
    Q = A(neighbour,:);
    
    %Finds points of equality between current and neighbouring solution to direct shifting
    Equal = find(P==Q); 
    
    %% Swapping wrt to neighbour
    
    % A swap can either move towards or away from a neighbouring solution
    % (increasing or decreasing similarity)
    
    % This checks that a move towards or away is possible 
    if( 2 < length(Equal) )&& ( length(Equal) < (length(P) - 2) ) 
        action = round(rand); %If so, pick a random motion direction
    else
        if(length(Equal)<2) % If all solution variables are different, move towards
            action = 1;
        else % If all solution variables are the same, move away
            action = 0;
        end
    end
    
    % R is a vector containing all points that can be swapped.
    if(action == 1)
        R = find(~(P==Q)); % Swap two different points to move towards
    else
        R = find(P==Q); % Swap two same points to move away
    end
    
    %Note: moving towards does not guarantee that the similarity between
    %the two solutions will increase, but it can never decrease. This means
    %over time, this operation will increase similarity. 
    
    
    %Select two indexes at random for swapping
    idxs = floor(rand(1,2)*length(R))+1;
    cut = 0;
 
    while P(R(idxs(1))) == P(R(idxs(2)))
       idxs(2) = floor(rand*length(R))+1;
       cut = cut + 1;
       %Ensure abnormal swap sets don’t freeze the code
       if(cut > 100)
           break;
       end
    end
 
    %Provided a valid set of indexes has been found, perform swapping
    if(cut < 100)
        [P(R(idxs(1))),P(R(idxs(2)))];
        %Swap
        tmp = A(i, (R(idxs(2))) );
        A(i, (R(idxs(2))) ) = P(R(idxs(1)));
        A(i, (R(idxs(1))) ) = tmp;
    end
    
    sol = A(i,:);
    
    %% Mutation
    mutation = 0.4; %Current mutation rate
     %1 / (size(Foods(i,:)) + 2);% %         mutation = 0.2; %1 / (size(Foods(i,:)) + 2);
     
    if(rand <= mutation) %In the case a solution is mutated,
       mutation = 0.1;
       %Each member has a 10% chance to be swapped to a random, non equal
       %value.
       for i=1:length(sol) 
           if(rand < mutation)
               sol(i) = sol(i) + round( (2*rand*(objs-1)) - 1);
               sol(i) = mod(sol(i),objs);
               if(sol(i) == 0) %As MATLAB indexes from 1.
                   sol(i) = sol(i) + objs;
               end
           end
       end
    end
end