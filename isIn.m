function [X] = isIn(a,b)
    for i = 1:length(b)
        if(a == b(i))
            X = true;
            return;
        end
    end
    X = false;
    return;
end