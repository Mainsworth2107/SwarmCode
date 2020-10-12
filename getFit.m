function X = getFit(input) %input is allocation array (1xN array)
    global others;
    knownBots = others;
    sum = 0; %Calculation variable

    %We need to have a set of known objects (knownObjs)
    %e.g (x,y,2;x,y,6 ...)

    %and a set of known robots (ID can be prioirty for this alg)
    %e.g (2,3,4)

    for i = 1:length(input)  
        if ~(input(i) == 0)
            for j = 1:height(knownBots{i}.knownObjs)
                if(knownBots{i}.knownObjs(j,3) == input(i))
                    sum = sum + knownBots{i}.knownObjs(j,1); %As this encodes disrance
                end
            end
        else
            sum = sum + 100;
        end
    end

    X = sum;
end 