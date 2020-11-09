function [X,Y] = newFit(poseIn)
    global allObjs
    costs = zeros(1,height(allObjs));
    %Control Parameters
    a = 1;b = 1;
    %test to frig results
%     a = 5;b = 1;
    vis = costs;
    qual = costs;
    total = 0;
    probs = costs;
    totQual = 0;
    for i = 1:length(costs)
        totQual = totQual + allObjs(i,4);
    end
    
    
    for i = 1:length(costs)
        test = poseIn;
        % Finds the EU distance element
        costs(i) = distEu(poseIn(1:2),allObjs(i,1:2)); 
        
        %Calculates visibility
        vis(i) = 1/costs(i);
        
        % Calculates quality
        qual(i) = allObjs(i,4) / totQual;
        
        total = total + ( (qual(i)^a) * (vis(i) ^ b)   );
    end
    
    for i=1:length(costs)
        p(i) = ((qual(i)^a) * (vis(i) ^ b)) / total;
    end
    
    choice = rand;
    total = 0;
    for i=1:length(p)
        total = total + p(i);
        if(total > choice)
            A = i;
            break
        end
    end
    X = A;
    Y = sum(costs);
end