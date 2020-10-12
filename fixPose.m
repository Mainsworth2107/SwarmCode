function X = fixPose(A)
%     X = asin(sin(A));
    while (A < 0)
        A = A + (2*pi);
    end
    if(A >= (2*pi))
        X = A - (floor(A / (2*pi)) * (2*pi));
    else
       X  = A;
    end
end