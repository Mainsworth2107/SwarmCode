function X = fixPose(A)
    if(A > 2*pi)
        X = A - (2 * pi * (A / abs(A)));
    else
        X = A;
    end
    return
end