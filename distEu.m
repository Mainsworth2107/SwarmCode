function [X] = distEu (A,B)
    sum = 0;
    for i = 1:length(A)
        sum = sum + (B(i) - A(i))^2;
    end
    sum = sqrt(sum);
    X = sum;
    return
end