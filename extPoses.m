%% Function that returns the poses of all robots in the environment
 
function X = extPoses(A)
    X = [];
    for i = 1:length(A)
        X(:,i) = A{i}.pose; %Gets and saves the pose for robot i.
    end
    return
end