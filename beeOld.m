function sol = beeOld(A,i,neighbour)
    test = A(i,:);
    global allObjs;
    objs = height(allObjs);
    
%     idxs = floor(rand*length(A(i,:)))+1;
    
    %Simple always towards shift
    idxs = floor(rand(1,1)*length(A(i,:)))+1;
%     cut = 0;
%     while(idxs(1) == idxs(2))
%        idxs(2) = floor(rand*length(A(i,:)))+1;
%        cut = cut + 1;
%        %Statement for saftey only
%        if(cut > 100)
%            break;
%        end
%     end
    
    for j=1:1
        if ~(A(i,idxs(j)) == A(neighbour,idxs(j)))
            A(i,idxs(j)) = mod( A(i,idxs(j)) + ceil(rand*(objs-1)) ,objs);
            if A(i,idxs(j)) == 0
                A(i,idxs(j)) = objs;
            end
        end
    end
    

    
    sol = A(i,:);
    
%     P = A(i,:);
%     Q = A(neighbour,:);
%     
%     Equal = find(P==Q);
%     %% Moving to neigbour
%     % Can move towards (1) or away (0)
%     if( 2 < length(Equal) )&& ( length(Equal) < (length(P) - 2) ) 
%         action = round(rand);
%     else
%         % All differnt
%         if(length(Equal)<2)
%             action = 1;
%         else %All Equal
%             action = 0;
%         end
%     end
%     
%     if(action == 1)
%         R = find(~(P==Q)); % Swap two differnt points to move towards
%     else
%         R = find(P==Q); % Swap two same points to move away
%     end
%     
%     idxs = floor(rand(1,2)*length(R))+1;
%     cut = 0;
% %     try
%     while P(R(idxs(1))) == P(R(idxs(2)))
%        idxs(2) = floor(rand*length(R))+1;
%        cut = cut + 1;
%        %Statement for saftey only
%        if(cut > 100)
%            break;
%        end
%     end
% %     catch
% %         waitfor(0.01)
% %     end
% 
%     if(cut < 100)
%         [P(R(idxs(1))),P(R(idxs(2)))];
%         %Swap
%         tmp = A(i, (R(idxs(2))) );
%         A(i, (R(idxs(2))) ) = P(R(idxs(1)));
%         A(i, (R(idxs(1))) ) = tmp;
%     end
% 
%     sol = A(i,:);
    %% Mutation
%     mutation = 0.4;
%      %1 / (size(Foods(i,:)) + 2);% %         mutation = 0.2; %1 / (size(Foods(i,:)) + 2);
%     if(rand <= mutation)
%        mutation = 0.1;
%        for i=1:length(sol)
%            if(rand < mutation)
%                sol(i) = sol(i) + round( (2*rand*(objs-1)) - 1);
%                sol(i) = mod(sol(i),objs);
%                if(sol(i) == 0)
%                    sol(i) = sol(i) + objs;
%                end
%            end
%        end
%     end
end