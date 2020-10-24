%% Task Allocation
function obj = greedyA(obj)
    %obj.getObjs();
    obj.knownObjs = obj.detObjs;
    if ~isempty(obj.knownObjs)
        %min = [-1,0,0]; %Furthest detected obj  
        %% Greedy Allocation being run
        % Min represents the final allocation
        min = [20,0,0];

        for j = 1:length(obj.knownObjs(:,1)) 
            if (obj.knownObjs(j,1) > 0) && (obj.knownObjs(j,1) < min(1))
                min = obj.knownObjs(j,:);
            end
        end

        obj.minT = min;

        obj.goal = ...
        [obj.pose(1) + (  min(1)*cos(min(2)) ), ...
         obj.pose(2) + (  min(1)*sin(min(2)) ), ...
         fixPose(min(2)), min(3)];

        obj.pose(3) = fixPose(obj.pose(3));
        obj.A = min(3);
        obj.state = 2;

    else %Defaults to resting if nothing seen
        obj.A = 0;
        obj.state = 4;               
    end
end