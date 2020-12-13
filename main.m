%% Acknowledgements
%
%% Gerneral Refernces
% Artificial Bee Colony algorithm
% Based on Materials Found at: https://abc.erciyes.edu.tr/ (MATLAB Code of the ABC algorithm version 2 )
%
% Orignal authors: 
% Dervis Karaboga (karaboga@erciyes.edu.tr )
% Bahriye Basturk Akay (bahriye@erciyes.edu.tr)
%
% Reference Papers as given by original authors:

%D. Karaboga, AN IDEA BASED ON HONEY BEE SWARM FOR NUMERICAL OPTIMIZATION,TECHNICAL REPORT-TR06, Erciyes University, Engineering Faculty, Computer Engineering Department 2005.*/

%D. Karaboga, B. Basturk, A powerful and Efficient Algorithm for Numerical Function Optimization: Artificial Bee Colony (ABC) Algorithm, Journal of Global Optimization, Volume:39, Issue:3,pp:459-171, November 2007,ISSN:0925-5001 , doi: 10.1007/s10898-007-9149-x */

%D. Karaboga, B. Basturk, On The Performance Of Artificial Bee Colony (ABC) Algorithm, Applied Soft Computing,Volume 8, Issue 1, January 2008, Pages 687-697. */

%D. Karaboga, B. Akay, A Comparative Study of Artificial Bee Colony Algorithm,  Applied Mathematics and Computation, 214, 108-132, 2009. */

% Uses The Mobile Robotics Simulation Toolbox (MRST) from: The MathWorks, Inc.
% Note: A small modification was made to the MRST code to draw the objects
% in the environment as slightly larger than the default size.%
%
%% RDABC Modifications
%
% Swapping and mutation, used in beeNew were based on the works of both:
%
% Deng et al: An Enhanced Discrete Artificial Bee Colony Algorithm to Minimize the Total Flow Time 
% in Permutation Flow Shop Scheduling with Limited Buffers
% found at:https://www.hindawi.com/journals/mpe/2016/7373617/
% and
% Badreldin: A Comparative Study between Optimization and Market-Based Approaches to Multi-Robot Task Allocation 
% found at: https://www.hindawi.com/journals/aai/2013/256524/
%
%
% The fitness function in beeFit.m is derived from the work of Jevtic et
% al: "Distributed Bees Algorithm Parameters Optimization for a Cost Efficient Target Allocation in Swarms of Robots"  
% found at:https://www.researchgate.net/publication/221676792_Distributed_Bees_Algorithm_Parameters_Optimization_for_a_Cost_Efficient_Target_Allocation_in_Swarms_of_Robots
%(page 6)
%
% The rSol method used for searching in shiftSol is based on the work of
% Sun et al : "An Artificial Bee Colony Algorithm with Random Location Updating"
% found at:https://www.hindawi.com/journals/sp/2018/2767546/
% 
%
%%
 
%N represents the number of robots (in comments)
%M represents the number of objects (in comments)
 
%% Create a multi-robot environment
flush
numRobots = 100; %Initialises the number of robots.

%Coefficient that controls total ABC iterations
if numRobots >= 100
    Coeff = 3; 
else
    Coeff = 2; 
end

runs = 1; % Total runs of the algorithm
 
env = MultiRobotEnv(numRobots); %Initialises robot envrionment (MRST)
env.showTrajectory = false; %Hides robot path information
 
% Limits of visualisation axes
limX = [-1, 1];
limY = [-1.4 1.4]; 
 
% limX = [-1.4 1.4];
% limY = [-1, 1];
 
A = zeros(1,numRobots); %Calculated allocation
 
%% Selecting testing scenario
 
% Setup 1
% objs = 2;                       % Number of Targets
% qualities = [0.5,0.5];          % Target qualities (q)
% preset = [-4.5,7.5; 4.5,-7.5];  % Object positions
 
% Setup 2
% objs = 4;
% qualities = 0.25*ones(1,4);
% preset = [-4.5,7.5; 4.5,-7.5;-4.5,-7.5; 4.5,7.5];
% % preset = [7.5,-4.5,;-7.5,4.5;-7.5,-4.5;7.5,4.5];
 
% Setup 3
objs = 4;
qualities = [0.1,0.2,0.3,0.4];
preset = [-4.5,7.5; 4.5,-7.5;-4.5,-7.5; 4.5,7.5];
% preset = [7.5,-4.5,;-7.5,4.5;-7.5,-4.5;7.5,4.5];
 
%Setup 4
% objs = 10;
% qualities = 0.1*ones(1,10);
% preset = [[4.5*ones(5,1);-4.5*ones(5,1)],[repmat([-7.5:(15/4):7.5]',2,1)]];
 
%Setup 5
% objs = 10;
% qualities = 0.02*[1:1:5,5:1:9];
% preset = [[4.5*ones(5,1);-4.5*ones(5,1)],[repmat([-7.5:(15/4):7.5]',2,1)]];
%
%% Adding Objects to environment
objects = zeros(objs,3); %Stores the coordinates of each object
colours = objects; %Sets the colour of each object
 
% Note the objects array also stores the object ID to account for situations
% where strict object ordering might not be possible or necessary.
 
objstr = 'o'; %Marker to draw the objects as circles.
 
for i = 1:objs %Initialises the objects within the environment
    objects(i,3) = i;
    
    %Positions are preset for the DBA
    objects(i,1:2) = preset(i,:) / 10;
    %objects(i,1:2) = [rand*limX(2),rand*limY(2)]; %Random object initialisation
 
    %Sets the necessary labels to draw all objects as circles
    if( i > 1)
        objstr = strcat(objstr,'o');
    end  
    
    %Makes all objects red
    colours(i,1) = 1;
    colours(i,2:3) = 0;
end
 
%Initialises global objects reference
global allObjs;
allObjs = [objects,(qualities')];
 
%Sets the necessary environment variables to draw the objects
env.hasObjects = 1;
env.objectColors = colours;
env.objectMarkers = objstr;
 
% Hides robot IDs
env.showRobotIds = false;
%% Initialises robots and arena
robots = cell(numRobots,1);
 
%Arena dimensions
diffX = 1.5;
diffY = 2.125; 
 
% diffX = 2.125;
% diffY = 1.5; 
 
%Function to initialise all robots
robots = initBots(robots,objs,diffX,diffY); 
 
% Allows for a sample robot position set to be loaded for specific tests
% save('robots.mat','robots'); 
% robots = load('robots.mat');
% robots = robots.robots;


% pain = load('pain.mat');
% pain = pain.pain;
% robots = pain{1};
%% Setting up the visualisation
%This process is slow, so can be skipped to save on processing time with
%larger problems.
 
%Extracts the robot poses
poses = extPoses(robots);
env.Poses =  poses;
 
%Draws the multi robot environment (this is an expensive operation, so not ran in loop). 
env(1:numRobots, poses, objects);
 
%Draws the robot arena boundaries.
line([diffX*0.5,diffX*-0.5],[diffY*0.5,diffY*0.5],'color','black','LineWidth',1); 
line([diffX*0.5,diffX*-0.5],[diffY*-0.5,diffY*-0.5],'color','black','LineWidth',1);
line([diffX*0.5,diffX*0.5],[diffY*0.5,diffY*-0.5],'color','black','LineWidth',1); 
line([diffX*-0.5,diffX*-0.5],[diffY*0.5,diffY*-0.5],'color','black','LineWidth',1); 
 
% Ensure that the visualisations axes remain fixed. 
% Without this, axis resizing can slow things down
xlim(limX);   
ylim(limY);
 
axis equal 
% Sets object labels
for i = 1:objs
    text(objects(i,1) - 0.005,objects(i,2) - 0.1,num2str(i),'Color',[0,0,0],'FontWeight','bold');
end
 
 
%% ABC Algorithm Setup
allFits = []; 
FoodNumber = 10; %Number of potential solutions being improved. (fixed at 10)
 
%A solution which could not be improved through "limit" 
% trials is abandoned by its employed bee
limit = 100; 
 
%Note: This algorithm optimises the allocations of robots that have not
%already found an object (as by definition, these robots cannot re allocate).
 
D = numRobots-objs; %The number of parameters of the problem to be optimized
 
%Assumption that required iterations are proportional to problem size
% maxCycle = Coeff*D; 
maxCycle = Coeff*numRobots; 
 
%Problem is bounded by the number of objects in the area
ub = ones(1,D)*objs; %Upper bounds of the parameters. 
lb = ones(1,D);%Lower bounds of the parameters.
 
%Store the best variables per cycle
GlobalMaxes = zeros(runs,D);
FitMaxes = zeros(1,runs);
 
%Tracks how quickly the optimum is reached 
%Note: very dependent on rng, not very helpful
coeffs = zeros(1,runs);
 
%Fitnesses and dist for each developed solution (in loop)
fitness = zeros(1,FoodNumber);
 
mae = zeros(1,runs); %MAE for each run
dists = mae;      %Total distance for each run
maxRobs = {};
 
%Used to enforce the bounds for the problem space
Range = repmat((ub-lb),[FoodNumber 1]);
Lower = repmat(lb, [FoodNumber 1]);

% pain = {};

%% Main Loop.
for i = 1:runs
%     pain{i} = robots;
%     robots = pain{i};
    tic;
% for i = 1:1
    %% ABC Algorithm
    % Input is robots
    
    %% Initialisation
    % 10 possible solutions  
    
%     Method 1: random
%     Randomly initialises the possible solutions (A), accounting for problem
%     bounds
%     Range = repmat((ub-lb),[FoodNumber 1]);
%     Lower = repmat(lb, [FoodNumber 1]);
%     A = (rand(FoodNumber,D) .* Range) + Lower;
    

%     Method 2: DBA
 
%Uses the DBA algorithm to initialise better solutions to be run through ABC
%     tests = floor(FoodNumber ./ 2);
% 
%     A = zeros(FoodNumber,numRobots);
%     tests = FoodNumber - 1; %The number of potential solutions to be produced by DBA
%     
%     for h = 1:tests %FoodNumber
%         for j = 1:numRobots %DBA Code
%             if(j <= objs) % If on an object, automatically allocate
%                 A(h,j) = j;
%             else %Otherwise, use DBA to allocate
%                 [P,Q] = newFit(robots{j}.pose);
%                 A(h,j) = P;
%             end
%         end
%     end
%     A = A(:,objs+1:end); %Reintegrate static robots
 
%   %Method 3: Greedy
%   Uses a greedy optimiser to produce high fitness initial solutions
 
%     Setup for pure greedy
    tests = 0;
    A = zeros(FoodNumber,D);
%     %Setup end
    
    %Greedy optimisation is handeled by an external function
    A(tests+1:end,:) = Greedy(tests,10,numRobots,robots,qualities);
   
    %Uses rounding to ensure all possible solutions consist of whole numbers
    A = round(A); 
    
    %Calculates the fitness values for each possible solution.
    fitness = beeFit(FoodNumber,A,robots,qualities,0);
    
    %Finds all the individuals with the best fitness value
    BestInd= find( fitness == max(fitness) );
    
    %Accounts for solutions each with the best fitness (not necessarily equal)
    BestInd=BestInd(end);
 
    % Saves the fitness and parameters of the best individual for later analysis
    GlobalMax = fitness(BestInd);
    GlobalParams = A(BestInd,:);
    sampleFits = [];
    fitDiffs = [];
    platCount = 0;
    
    %Resets trail array
    trial=zeros(1,FoodNumber); %reset trial counters
    iter=1; %Resets iteration count
    %     sampleFits = zeros(numRobots-objs,FoodNumber); %Testing variable
    
    %% Main Iteration Loop
    while ((iter <= maxCycle))
        %% Employed Bee Phase
        for j=1:(FoodNumber) % For each employed bee ...
            
            % Mutates the input solution
            sol = shiftSol(A,FoodNumber,j);
            
            %Determines is the mutated solution is better
            [fitness(j),trial(j)] = ...
                updateSol(sol,fitness(j),trial(j),robots,qualities);
            
            %If the mutated solution was better, update the solution set
            if trial(j) == 0 
                A(j,:) = sol;
            end
        end
        
        % Once all 10 fitness values are known, normalise them to
        % probabilities in the range (0.1 - 1).
        
        prob=(0.9.*fitness./max(fitness))+0.1;
        
        %% Onlooker Bee Phase
        j = 1; %Looping variable
        t=0;   
        
        while(t<FoodNumber) %Until 10 onlooker bees have selected a target
            if(rand<prob(j)) % If a solution is selected
                t=t+1; %Increase selected solution count
 
                sol = shiftSol(A,FoodNumber,j); %Mutate the selected solution
 
                %Determines is the mutated solution is better
                [fitness(j),trial(j)] = ...
                    updateSol(sol,fitness(j),trial(j),robots,qualities);
 
                %If the mutated solution was better, update the solution set
                if trial(j) == 0
                    A(j,:) = sol;
                end     
            end
 
            % All possible solutions are looped through until 10 are chosen.
            j=j+1;
            if (j==(FoodNumber)+1) 
                j=1;
            end 
        end
        
        %Finds the best current individual
        ind=find(fitness==max(fitness));
 
        %Accounts for the possibility of more than 1 solution sharing the maximum fitness
        ind=ind(end); 
        
        %If the current best value is better than the global best, update
        %the global best value
        if (fitness(ind)>GlobalMax)
            GlobalMax=fitness(ind);
            GlobalParams=A(ind,:);
            coeffs(i) = iter;
        end
         
        %Record how the global max changes over each run
%         dists(iter) = GlobalMax;
        
        %% Scout Bee Phase
        
        %Only one solution can be scouted per iteration
        toChange=find(trial==max(trial));
        toChange=toChange(end);
        
        Glim = -1;
%         Glim = -1;
        % Global pullup removal (unused atm)
        if(iter == Glim)
            toChange = ind;
        end
        
        % If the maximum trail value is greater than the limit, reset the
        % counter and choose a new solution for that individual.
        if ((trial(toChange)>limit) && ~(toChange==ind)) || (iter==Glim)
            
            %This prevents multiple solutions from being scouted in
            %sequential loops
%             trial = trial - 25;
            
            trial(toChange)=0;
            
            %Avoids negative trail counts
            if ~isempty(find(trial<0))
                trial(trial<0) = 0;
            end
            
            % Scouted solutions are generated randomly
            sol = (rand(1,D) * Range(toChange)) + Lower(toChange);
            sol = round(sol);
            
            
            % Updates the fitness values to include the new solution
            FitnessSol=beeFit(1,sol,robots,qualities,0);
            
            A(toChange,:)=sol;
            fitness(toChange)=FitnessSol;
        end
        
%         if(mod(iter,Coeff) == 0)
%             sampleFits(iter/Coeff,:) = fitness;
%         end
 
        % Record all ten fitness values for plotting results.
        sampleFits(iter,:) = fitness;
        if(iter > 1)
            fitDiffs(iter-1,:) = sampleFits(iter,:) - sampleFits(iter-1,:);
        end
        
%         if(iter > 10)
%             if(length(find(fitDiffs(iter-1,:)==0)) >= 8)
%                 platCount = platCount +1;
%             else
%                 platCount = 0;
%             end
%             if(platCount >=8)
%                 break;
%             end
%         end
        iter = iter + 1;
    end
    
%% End of Loop Recording
    cuts(i) = iter;
    times(i) = toc;
    %Saves the overall maximum recording and parameters
    GlobalMaxes(i,:) = GlobalParams;
    FitMaxes(i) = GlobalMax;
    
    if(FitMaxes(i) == max(FitMaxes))
        maxGMax = i;
        maxRobs = robots;
    end
    
   %Records the mae for each loop
    mae(i) = beeFit(1,GlobalParams,robots,qualities,1);
    
    %Record the total distance for each loop
    dists(i) = beeFit(1,GlobalParams,robots,qualities,2);
    
    if(i < (runs))
        robots = initBots(robots,objs,diffX,diffY); %Sets a New robots array
    end
%     figure(4)
%     plot(sampleFits)
    disp(['Run: ',num2str(i),' of ',num2str(runs)]);
%     hold on
%     figure(3)
%     plot(sampleFits)
%     waitfor(0.01)
%     allFits = [allFits,sampleFits];
end
%%
% histogram(allFits,50);
% allFits = allFits(:);
% allFits = allFits(find(allFits >=183.6));

%%
% Plots how the fitness for all 10 potential solutions evolved through the
% % last run.
figure(3)
if(iter <= maxCycle)
    plot(sampleFits(1:iter,:))
else
    plot(sampleFits)
end

xlabel('Iterations')
ylabel('Fitness')
ylim([0,FitMaxes(end)*1.25])

figure(4)
plot(fitDiffs)
%Absolute best run analysis
% Extends the absolute best allocation set to include static robots for plotting.
% sample = [1:objs,GlobalMaxes(end,:)];
% robots = maxRobs;
 
%Last run analysis
% Extends the final allocation set to include static robots for plotting.
sample = [1:objs,GlobalParams];
 
 
%% Producing an example visualisation
%Initialising main environment visualisation
figure(1)
poses = extPoses(robots);
env.Poses =  poses;
env(1:numRobots, poses, objects);
xlim(limX);   % Without this, axis resizing can slow things down
ylim(limY);  
 
%  % %Sets object labels
% for i = 1:objs
%     text(objects(i,1) - 0.05,objects(i,2) - 0.3,num2str(i),'Color',[0,0,0],'FontWeight','bold');
% end

% figure(1)
% sample = [1:objs,GlobalParams];
% sample = [1,2,opt];
% sample = A(ceil(rand*height(A)),:);
 
% Extracts the robot pose set
poses = extPoses(robots);
 
%Draw the example robot positions.
env.Poses =  poses;
env(1:numRobots, poses, objects);
 
%Draws lines between each robot and its respective line to visually
%represent the example allocation.
for i = objs:numRobots
    line([robots{i}.pose(1),objects(sample(i),1)],...
         [robots{i}.pose(2),objects(sample(i),2)],...
         'color','black','LineWidth',1);
end
 
 
% %% Bar chart comparing average allocation across all runs to desired allocation
%  
% %Counts the total number of robots allocated to each task, then normalises
% %the results
% tmp = histcounts(sample,'Normalization','probability');
% tmp = tmp*100;
%  
% %Finds the visual error between the bars (not necessarily equal to
% %MAE)
% visE = abs(tmp(1)-tmp(2))/2;
%  
% %X axis for the bar chart 
% x = 1:length(tmp);
%  
% %Y axis for the bar chart
% for i =1:length(tmp)
%     y(i,1) = tmp(i);
%     y(i,2) = 100*(qualities(i)/ sum(qualities));
% end
%  
% % Showing the bar chart
% figure(2);
%  
% tmp = bar(x,y,0.75);
%  
% %Sets the colour for the obtained distribution (cyan)
% tmp(1).FaceColor = [0 1 1]; 
%  
% %Sets the colour for the obtained distribution (green)
% tmp(2).FaceColor = [0 1 0]; 
%  
% %Sets up the legend labels for the bar chart
% set(tmp, {'DisplayName'}, {'Obtained','Expected'}');
%  
% %Axes labels for the bar chart
% xlabel('Target');
% ylabel('Number of robots (%)');
% ylim([0 50]); %0.7 1, 0.4 2, 0.5 3
%  
% % Showing the legend for the bar chart
% legend('Location','northwest')
%  
% %% Calculation of results
%  
% %Calculating the maximum and average MAE
% maxE = max(mae);
% avgE = mean(mae);
%  
% %Displaying the average and maximum mae alongside total distance
% %Note that mae is shown in % and distance is shown in m.
% disp('   AvgE(%)   MaxE(%)   Avg Dist');
% disp([100*avgE 100*maxE (mean(dists))]);
%%
% Writing results to csv for validation
% out = [GlobalMaxes,mae',allDists'];
% %% Writing results to csv for validation   1000*
% writematrix([((objs / 2)*numRobots*mae'),dists',FitMaxes',times'],'Test.csv');