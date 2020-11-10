%% Distributed bees aglrotihm
%% Based on:
%% Uses The Mobile Simulation Toolbox from: The MathWorks, Inc.


%% Create a multi-robot environment
flush
runs = 1;
numRobots = 40; %Self explanitary
% Coeff = 2; %Looping coefficent (Approriate measure)
Coeff = 6;
%The looping coefficent is representative of the number of iterations
%required to develop an optimal soultion. As such, the goal of
%optimisiation is minimising this multiplier.

env = MultiRobotEnv(numRobots); %Initalises robot envrionment
env.showTrajectory = false; %Disables live pathing
% env.robotRadius = 0.25; %Determines robot chunkiness

limX = [-14 14]; % 8
limY = [-10, 10];

A = zeros(1,numRobots-2); %Allocation

%% Initalising Objects

%setup 1
% objs = 2;
% qualities = [0.5,0.5];
% preset = [-4.5,7.5; 4.5,-7.5];

%setup 2
% objs = 4;
% qualities = 0.25*ones(1,4);
% preset = [-4.5,7.5; 4.5,-7.5;-4.5,-7.5; 4.5,7.5];

% %setup 3
objs = 4;
qualities = [0.1,0.2,0.3,0.4];
preset = [-4.5,7.5; 4.5,-7.5;-4.5,-7.5; 4.5,7.5];

%% Adding Objects to environment
objects = zeros(objs,3);
colours = objects;

%Stores the detection data of an object.
detected = [[],[]];
%preset = [2,2;2,14;14,2;14,14]; %Must be the same size as all objects
% ;-4.5,-7.5; 4.5,7.5]; %];

objstr = 'o';
for i = 1:objs
    objects(i,3) = i;
    objects(i,1:2) = preset(i,:);
    %objects(i,1:2) = [rand*limX(2),rand*limY(2)];

    % Sets the necessary labels to draw all objects as circles
    if( i > 1)
        objstr = strcat(objstr,'o');
    end  
    %Makes all objects red
    colours(i,1) = 1;
    colours(i,2:3) = 0;
end

%Initalises global objects reference
global allObjs;
allObjs = [objects,(qualities')];

env.hasObjects = 1;
env.objectColors = colours;
env.objectMarkers = objstr;

% Hides robot IDs
% env.showRobotIds = false;

%% Initalises arena and robots

% Arena dimensions
diffX = 21.25;
diffY = 15;

%This is the input scenario being optimised
% (for each scario the enitre optimisation process is re ran)

% robots = cell(numRobots,1);
% robots = initBots(robots,objs,diffX,diffY);


% save('robots.mat','robots'); 
robots = load('robots.mat');
robots = robots.robots;
%% Setting up the visulisation

% %Initalising main envrionment visu
% poses = extPoses(robots);
% env.Poses =  poses;
% env(1:numRobots, poses, objects);
% xlim(limX);   % Without this, axis resizing can slow things down
% ylim(limY);  
% 
% %Sets ojbect labels
% for i = 1:objs
%     text(objects(i,1) - 0.05,objects(i,2) - 0.3,num2str(i),'Color',[0,0,0],'FontWeight','bold');
% end


%% Bees setup
% Lines 1 - 25

FoodNumber = 10; %Number of continous soltions (fixed at 10)
limit = 80; %A food source which could not be improved through "limit" 
%   trials is abandoned by its employed bee*/

%Note: This only allocates the N-2 robots not already allocated

D = numRobots-objs; %/*The number of parameters of the problem to be optimized*/
maxCycle = Coeff*D; %Assumption that req its is directly proportianl to problem size

%Probelm is bounded by the number of objects in the area
ub = ones(1,D)*objs; %/*upper bounds of the parameters. */
lb = ones(1,D);%/*lower bounds of the parameters.*/

%Store the best variables per cycle
GlobalMaxes = zeros(runs,D);
FitMaxes = zeros(1,runs);
mae = zeros(1,runs); %MAE for each run
allDists = mae;

%Tracks how quickly the optimum is reached
coeffs = zeros(1,runs);

%Fitnessess and dist for each developed solution (in loop)
fitness = zeros(1,FoodNumber);
dists = fitness;

% opt = [1,2,2,2,2,2,1,2....
%        2,2,1,1,1,1,2,2,1,1 ...
%        2,1,1,2,1,1,1,2,2,1 ...
%        1,1,2,1,1,2,2,1,2,1];

%% Main Loop.
for i = 1:runs
% for i = 1:1
    %% ABC Algortihm
    % Input is robots
    
    %% Initalisation
    % 10 possible solutions
    Range = repmat((ub-lb),[FoodNumber 1]);
    Lower = repmat(lb, [FoodNumber 1]);
%     tests = floor(3*FoodNumber ./ 4);
    tests = 9;
%     Method 1: random
%     A is the solutions matrix, and is randomly initalised
%     A = (rand(FoodNumber,D) .* Range) + Lower;
    
%     Method 2: DBA
    Counter = zeros(1,objs);
    A = [];
    for h = 1:tests %FoodNumber
%     for h = 1:FoodNumber
        for j = 1:numRobots
            if(j <= objs)
                A(h,j) = j;
            else
                [P,Q] = newFit(robots{j}.pose);
                % MAE does not consider already allocated robots.
                A(h,j) = P;
            end
        end
    end
%     A = A(:,objs+1:end);
%     %Method 3: Greedy
% 
    for h = tests+1:FoodNumber
%     for h = 1:FoodNumber
%         test = zeros(1,numRobots);
        Dists = zeros(numRobots,objs);
        for j = 1:numRobots
            for k = 1:objs
                Dists(j,k) = distEu(robots{j}.pose(1:2),objects(k,1:2));
            end
            Dists(j,objs+1) = j;
        end
        Dists = sortrows(Dists);

        left = numRobots;
        On = 0;
        qualDyn = qualities;
        for j = 1:objs
            props = round(left * qualities(1));
            A(h,Dists( (On + 1):(On+props) ,end)') = j;
            Dists = Dists(props+1:end,2:end);
            Dists = sortrows(Dists);
            %%

            left = left - props;
            tmp = qualities(1);
            qualities = qualities(2:end);
            qualities = qualities ./ (1-tmp);
        end
    qualities = qualDyn;
    end
    A = A(:,objs+1:end);
    %%
    
    A = round(A);
    
    %Calculate current fitnesses.
    test = beeFit(FoodNumber,A,robots,qualities,1);
    fitness = beeFit(FoodNumber,A,robots,qualities,0);
    
    %Finds all the indviduals with the best value
    BestInd= find( fitness == max(fitness) );
    % Arbitarily selects the last individual in the swarm with that value
    BestInd=BestInd(end);

    % Saves the fitness and parameters of the best individual
    GlobalMax = fitness(BestInd);
    GlobalParams = A(BestInd,:);
    
    %Resets trail array
    trial=zeros(1,FoodNumber); %reset trial counters
    
    %% Big Loop
    if(i == 20)
        waitfor(0.01)
    end
    iter=1;
%     sampleFits = zeros(numRobots-objs,FoodNumber); %Testing variable
    gTrials = 0;
    
    while ((iter <= maxCycle))
        %% Employed Bee Phase
        for j=1:(FoodNumber) % Exactly [solutions] runs completed
            
%             tmp = A(j,:); % Testing value
            
            % Determines a shiifted solution
            sol = shiftSol(A,FoodNumber,j);
%             length(find(~(tmp==sol)))
            
            %Determines is the altered solution is better
            [fitness(j),trial(j)] = ...
                updateSol(sol,fitness(j),trial(j),robots,qualities);
            
            if trial(j) == 0 %Uses the fact that trials reset when a better solution is found
                A(j,:) = sol;
            end
        end
        
        %Solutions have p in range 0.1 to 1
        prob=(0.9.*fitness./max(fitness))+0.1;
             
        %% Onlooker Bee Phase
        j = 1; %Looping variable
        t=0;
        while(t<FoodNumber)
            if(rand<prob(j))
                t=t+1;
                sol = shiftSol(A,FoodNumber,j);

                %Determines is the altered solution is better
                [fitness(j),trial(j)] = ...
                    updateSol(sol,fitness(j),trial(j),robots,qualities);

                if trial(j) == 0
                    A(j,:) = sol;
                end     
            end

            j=j+1;
            
            if (j==(FoodNumber)+1) 
                j=1;
            end 
            
        end
        
        %/*The best food source is memorized*/
         ind=find(fitness==max(fitness));
         ind=ind(end);
         if (fitness(ind)>GlobalMax)
            GlobalMax=fitness(ind);
            GlobalParams=A(ind,:);
            coeffs(i) = iter;
            gTrails = 0;
         else
%              gTrials = gTrials + 1;
%             if gTrials > 50
%                 break;
%             end
         end
         
        dists(iter) = GlobalMax;%1/((testing + (1 /numRobots))*GlobalMax);
        % Scout Bee Phase
        
        toChange=find(trial==max(trial));
        toChange=toChange(end);
        if(iter == 3)
            toChange = ind;
        end
        
        if (trial(toChange)>limit)   %% && ~(toChange==ind)) || (iter==5)
            trial = trial - 25;
            trial(toChange)=0;
            if ~isempty(find(trial<0))
                trial(trial<0) = 0;
            end
            sol = (rand(1,D) * Range(toChange)) + Lower(toChange);
            sol = round(sol);

            
%             sol = A(toChange,:);            
%            mutation = 0.5;
%            for j=1:length(sol)
%                if(rand < mutation)
%                    sol(j) = sol(j) + round( (2*rand*(objs-1)) ); %- 1
%                    sol(j) = mod(sol(j),objs);
%                    if(sol(j) == 0)
%                        sol(j) = sol(j) + objs;
%                    end
%                end
%            end
            
            FitnessSol=beeFit(1,sol,robots,qualities,0);
            
            A(toChange,:)=sol;
            fitness(toChange)=FitnessSol;
        end
        
%         if(mod(iter,Coeff) == 0)
%             sampleFits(iter/Coeff,:) = fitness;
%         end

        sampleFits(iter,:) = fitness;
        iter = iter + 1;
        if(iter > (maxCycle /2))
            waitfor(0.01);
        end
    end
    
    %% End of Loop Recording
    GlobalMaxes(i,:) = GlobalParams;
    FitMaxes(i) = GlobalMax;
    mae(i) = beeFit(1,GlobalParams,robots,qualities,1);
    allDists(i) = beeFit(1,GlobalParams,robots,qualities,2);
    if(i < (runs))
        robots = initBots(robots,objs,diffX,diffY); %Sets a New robots array
    end
%     figure(4)
%     plot(sampleFits)
    Vals(i) = max(sampleFits(end,:));
    waitfor(0.01)
end
    
% coeffs = coeffs ./ numRobots;
figure(3)
plot(sampleFits)

%% 


% sample = [1:objs,GlobalMaxes(end,:)];
sample = [1:objs,GlobalParams];


%% Producing an example visulisation
% %Initalising main envrionment visu
poses = extPoses(robots);
env.Poses =  poses;
env(1:numRobots, poses, objects);
xlim(limX);   % Without this, axis resizing can slow things down
ylim(limY);  

%Sets ojbect labels
for i = 1:objs
    text(objects(i,1) - 0.05,objects(i,2) - 0.3,num2str(i),'Color',[0,0,0],'FontWeight','bold');
end

figure(1)
% sample = [1:objs,GlobalParams];
% sample = [1,2,opt];
% sample = A(ceil(rand*height(A)),:);
poses = extPoses(robots);

for i = objs:numRobots
    line([robots{i}.pose(1),objects(sample(i),1)],...
         [robots{i}.pose(2),objects(sample(i),2)],...
         'color','black','LineWidth',1);
end
env.Poses =  poses;
env(1:numRobots, poses, objects);


%% Bar Chart
% Adds fixed pos robots to results
fixed = ones(runs,1)*(1:objs);
% tmp = histcounts(sample,'Normalization','probability');
tmp = histcounts([fixed,GlobalMaxes],'Normalization','probability');
tmp = tmp*100;
visE = tmp;
x = 1:length(tmp);

for i =1:length(tmp)
    y(i,1) = tmp(i);
    y(i,2) = 100*(qualities(i)/ sum(qualities));
end

figure(2);
tmp = bar(x,y,0.75);
tmp(1).FaceColor = [0 1 1];
tmp(2).FaceColor = [0 1 0];
set(tmp, {'DisplayName'}, {'Obtained','Expected'}');
xlabel('Target');
ylabel('Number of robots (%)');
ylim([0 70]);

legend('Location','northwest')
% %%
% figure(3)
% plot(coeffs)
% %%
% maxE = max(mae);
% avgE = mean(mae);
% 
% %% Output of results
% format short
% disp('   AvgE(%)   MaxE(%)   Avg Dist');
% % disp('    AvgE      MaxE');
% disp([100*avgE 100*maxE (mean(allDists)/10)]);
% 
% out = [GlobalMaxes,mae',allDists'];
% %% Writing results to csv for validation
% writematrix((out),'Test.csv');

