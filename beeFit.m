function X = beeFit(FoodNumber,A,robots,qualities)
    %Fitness calclation (for each robot)
    global allObjs
    objs = height(allObjs);
    X = zeros(1,FoodNumber);
    
    for i = 1:FoodNumber
        %Count and dist are the in loop verisons of counts and dists
        
        counts = zeros(1,objs); %Allocation counts for each object in the space
        dist = 0;
        % Count allocations to each robot
        for k = 1:objs
            counts(k) = length(find( A(i,:)==k ));
        end

        
        for k = 1:length(A(i,:)) %First objs robots ignored
            dist = dist + distEu(robots{k+objs}.pose(1:2), allObjs(A(i,k),1:2) );
        end
        
        %Averages the distance across all robots
        dist = dist / length(robots);
        
        counts = counts + 1; %Adding static robots;
        counts = counts ./ (sum(counts));
        
        %error is the in loop error
        Error = (1/objs) * sum(abs(qualities - counts));
        
        %As defined in a follow up paper fitness = 1/(mae*dist)

        %To avoid infs everywhere, the MAE is given a statistically
        %insignificant bais (minimum for 100 robots is 0.01 (1000*bias)).

        %mae is of greater importance than distance because it has a smaller
        %(larger inverse) order of magnitude. This is good as we don't really
        %care about distance until MAE is zero.

        X(i) = (1/  ((Error + (0.001/length(robots)) ) * dist ) );
%         X(i) = (1/  (Error + (0.001/length(robots)) ));
%           X(i) = 1/dist;
%          X(i) = (1/ ((Error) * dist)); %Produces infs
    end
    
end