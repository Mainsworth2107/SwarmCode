function [X] = distEu (A,B)
    sum = 0;
    for i = 1:size(A)
        sum = sum + sqrt((B(i) - A(i))^2);
    end
    X = sum;
    return
end