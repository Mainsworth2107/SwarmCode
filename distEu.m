%% A function to calculate the Euclidian distance between any two ND points
 
function [X] = distEu (A,B)
    sum = 0;
    for i = 1:length(A) % Sums all of the squares for dimension i
        sum = sum + (B(i) - A(i))^2;
    end
    %Finds the square root across all summed dimension squares.
    sum = sqrt(sum);
    X = sum;
    return
end