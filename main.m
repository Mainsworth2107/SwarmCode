%% Distributed bees aglrotihm
%% Based on:
%% Uses The Mobile Simulation Toolbox from: The MathWorks, Inc.


%% Create a multi-robot environment
flush
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

A = zeros(1,numRobots); %Allocation

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

robots = cell(numRobots,1);
robots = initBots(robots,objs,diffX,diffY);


% save('robots.mat','robots'); 
% robots = load('robots.mat');
% robots = robots.robots;
%% Setting up the visulisation

%Initalising main envrionment visu
poses = extPoses(robots);
env.Poses =  poses;
env(1:numRobots, poses, objects);
xlim(limX);   % Without this, axis resizing can slow things down
ylim(limY);  

%Sets ojbect labels
for i = 1:objs
    text(objects(i,1) - 0.05,objects(i,2) - 0.3,num2str(i),'Color',[0,0,0],'FontWeight','bold');
end

%% Running the algortihm
% runs = 50;
runs = 1;
mae = zeros(1,runs); %MAE for each run
% dists = mae; %Sum of distances for each run


%% Bees setup
% Lines 1 - 25

FoodNumber = 10; %Number of continous soltions (fixed at 10)
limit = 100; %A food source which could not be improved through "limit" 
%   trials is abandoned by its employed bee*/

%Note: This only allocates the N-2 robots not already allocated

D = numRobots-objs; %/*The number of parameters of the problem to be optimized*/
maxCycle = Coeff*D; %Assumption that req its is directly proportianl to problem size

%Probelm is bounded by the number of objects in the area
ub = ones(1,D)*objs; %/*upper bounds of the parameters. */
lb = ones(1,D);%/*lower bounds of the parameters.*/

GlobalMaxes = zeros(1,runs);

testing = zeros(1,objs);

%Fitnessess of all individuals over all loops
fitness = zeros(runs,FoodNumber);
dists = zeros(1,runs);

% opt = [1,2,2,2,2,2,1,2....
%        2,2,1,1,1,1,2,2,1,1 ...
%        2,1,1,2,1,1,1,2,2,1 ...
%        1,1,2,1,1,2,2,1,2,1];

%% Main Loop.
for i = 1:runs
    % Input is robots
    
    %% ABC Algortihm
    
    %% Initalisation
    % 10 possible solutions
    Range = repmat((ub-lb),[FoodNumber 1]);
    Lower = repmat(lb, [FoodNumber 1]);
    % A is the solutions matrix
    A = (rand(FoodNumber,D) .* Range) + Lower;
    A = round(A);
    
    %Calculate current fitnesses.
    fitness = beeFit(FoodNumber,A,robots,qualities);
    
    trial=zeros(1,FoodNumber); %reset trial counters

    %Finds all the indviduals with the best value
    BestInd= find( fitness == max(fitness) );
    % Arbitarily selects the last individual in the swarm with that value
    BestInd=BestInd(end);

    % Saves the fitness and parameters of the best individual
    GlobalMax = fitness(BestInd);
    GlobalParams = A(BestInd,:);
    
    %% Big Loop
    iter=1;
    sampleFits = zeros(numRobots,FoodNumber);
    
    
    while ((iter <= maxCycle))
        %% Employed Bee Phase
        for j=1:(FoodNumber) % Exactly [solutions] runs completed
            
            % Determines a shiifted solution
            tmp = A(j,:); % Testing value
            sol = shiftSol(A,FoodNumber,j);
%             length(find(~(tmp==sol)))
            
            %Determines is the altered solution is better
            [fitness(j),trial(j)] = ...
                updateSol(sol,fitness(j),trial(j),robots,qualities);
            
            if trial(j) == 0
                A(j,:) = sol;
            end
        end
        
        prob=(0.9.*fitness./max(fitness))+0.1;
             
        %% Onlooker Bee Phase
        j = 1;
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
         end
        
%         for k = 1:objs
%             testing(k) = length(find( GlobalParams==k ));
%         end
%         testing = testing + 1; %Adding static robots;
%         testing = testing ./ (sum(testing));
%         testing = (1/objs) * sum(abs(qualities - testing));
        
        dists(iter) = GlobalMax;%1/((testing + (1 /numRobots))*GlobalMax);
        %% Scout Bee Phase
        
        ind=find(trial==max(trial));
        ind=ind(end);
        
        if (trial(ind)>limit)
            trial(ind)=0;
            
            sol = (rand(1,D) * Range(ind)) + Lower(ind);
            sol = round(sol);

            FitnessSol=beeFit(1,sol,robots,qualities);
            
            A(ind,:)=sol;
            fitness(ind)=FitnessSol;
        end
        
        if(mod(iter,Coeff) == 0)
            sampleFits(iter/Coeff,:) = fitness;
        end
         
        
        iter = iter + 1;
    end
end

figure(2)
plot(dists)

%% 


%     %% Assess the fitness retunred by the allocation
%     % MAE considers already allocated robotos
%     
%     counts = counts ./ (sum(counts));
%     mae(i) = (1/objs) * sum(abs(qualities - counts));
%     
% 
%     
%     
%     %Initalises next run
%     robots = initBots(robots,objs,diffX,diffY);
% end

%% Producing an example visulisation
figure(1)
sample = [1:objs,GlobalParams];
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

tmp = histcounts([1:objs,GlobalParams],'Normalization','probability');
tmp = tmp*100;
visE = tmp;
x = 1:length(tmp);

for i =1:length(tmp)
    y(i,1) = tmp(i);
    y(i,2) = 100*(qualities(i)/ sum(qualities));
end

figure(3);
tmp = bar(x,y,0.75);
tmp(1).FaceColor = [0 1 1];
tmp(2).FaceColor = [0 1 0];
set(tmp, {'DisplayName'}, {'Obtained','Expected'}');
xlabel('Target');
ylabel('Number of robots (%)');
ylim([0 70]);

legend('Location','northwest')

% %% Error Max and average
% 
% maxE = max(mae);
% avgE = mean(mae);
% visE = abs(visE(1)-visE(2))/2;
% 
% %% Output of results
% format short
% disp('   AvgE(%)   MaxE(%)   Avg Dist');
% % disp('    AvgE      MaxE');
% disp([100*avgE 100*maxE (mean(dists)/10)]);
% 
% out = [floor(A),mae',dists'];
% %% Writing results to csv for validation
% writematrix((out),'Test.csv');

