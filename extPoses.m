function X = extPoses(A)
    X = [];
    for i = 1:length(A)
        X(:,i) = A{i}.pose; 
    end
    return
end