%% Function that manages the greedy allocation of a given number of solutions

function [A] =  greedy(start,foodNumber,numRobots,robots,qualities)   
    global allObjs;
    objects = allObjs;
    objs = height(allObjs);
    A = zeros(foodNumber-start,numRobots);
    for h = start:foodNumber
        %Finds the distance from every non static robot to each object
        Dists = zeros(numRobots-objs,objs);
        for j = objs+1:numRobots
            for k = 1:objs
                Dists(j-objs,k) = distEu(robots{j}.pose(1:2),objects(k,1:2));
            end
            Dists(j-objs,objs+1) = j;
        end

        %By randomising the order objects get allocated, greedy can produce
        %vairable allocations
%         Order = [1:objs];
        Order = randperm(objs);
        %Sorts the distance array according to distance to object h
    %         Dists = Dists
        Dists(:,1:objs) = Dists(:,Order);
        Dists = sortrows(Dists);

        %Initialises values for greedy loop
        On = 0;

        % Greedy Allocation Loop
        for j = 1:objs

            waitfor(0.01)

            %Uses first come first served allocation (objects)
    %             props = round(left * qualities(1));
            %As we want one less due to statics
            props = round(numRobots * qualities(Order(j))) - 1;
            %Allocates the desired number of closest robots
            try
                A(h-start,Dists( (On + 1):(On+props) ,end)') = Order(j);
            catch
                waitfor(0.01)
            end
            %Removes allocated robots from stack
            Dists = Dists(props+1:end,2:end);
            Dists = sortrows(Dists);
        end
    end
    A = A(:,objs+1:end);     