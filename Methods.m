%     Method 2: DBA
%     Counter = zeros(1,objs);
%     A = [];
% %     for h = 1:tests %FoodNumber
%     for h = 1:FoodNumber
%         for j = 1:D
%             if j <= objs
%                 A(h,j) = j;
%                 Counter(A(h,j)) = Counter(A(h,j)) + 1;
%             else
%                 [P,Q] = newFit(robots{j}.pose);
%                 % MAE does not consider already allocated robots.
%                 A(h,j) = P;
%             end
%         end
%     end
%     A = [ones(tests,objs),A];
%     %Method 3: Greedy
% 
%     for h = tests+1:FoodNumber
% %         test = zeros(1,numRobots);
%         Dists = zeros(numRobots,objs);
%         for j = 1:numRobots
%             for k = 1:objs
%                 Dists(j,k) = distEu(robots{j}.pose(1:2),objects(k,1:2));
%             end
%             Dists(j,objs+1) = j;
%         end
%         Dists = sortrows(Dists);
% 
%         left = numRobots;
%         On = 0;
%         qualDyn = qualities;
%         for j = 1:objs
%             props = round(left * qualities(1));
%             A(h,Dists( (On + 1):(On+props) ,end)') = j;
%             Dists = Dists(props+1:end,2:end);
%             Dists = sortrows(Dists);
%             %%
% 
%             left = left - props;
%             tmp = qualities(1);
%             qualities = qualities(2:end);
%             qualities = qualities ./ (1-tmp);
%         end
%     qualities = qualDyn;
%     end
%     A = A(:,objs+1:end);