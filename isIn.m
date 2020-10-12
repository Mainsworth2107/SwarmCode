function [X] = isIn(a,b)
    if ~(isempty(a) && isempty(b))
        for i = 1:length(b)
            if(a == b(i))
                X = true;
                return;
            end
        end
    end
    X = false;
    return;
end