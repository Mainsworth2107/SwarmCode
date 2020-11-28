%% Function that returns the height of a 2D matrix
% Note: it turns out the length function does this if you add a second
% argument of 1.
function X  = height(A)
    tmp = size(A);
    X = tmp(1);
end