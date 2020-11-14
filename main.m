%% Acknowledgements
%
% Artificial Bee Colony algorithm
% Based on Materials Found at: https://abc.erciyes.edu.tr/
% Uses The Mobile Robotics Simulation Toolbox (MRST) from: The MathWorks, Inc.
% Note: A small modification was made to the MRST code to draw the objects
% in the environment as slightly larger than the default size.
 
%N represents the number of robots (in comments)
%M represents the number of objects (in comments)
 
%% Create a multi-robot environment
flush
numRobots = 20; %Initialises the number of robots.
 
Coeff = 6; %Coefficient that controls total ABC iterations
runs = 50; % Total runs of the algorithm
 
env = MultiRobotEnv(numRobots); %Initialises robot envrionment (MRST)
env.showTrajectory = false; %Hides robot path information
 
% Limits of visualisation axes
% limX = [-1, 1];
% limY = [-1.4 1.4]; 
 
limX = [-1.4 1.4];
limY = [-1, 1];
 
A = zeros(1,numRobots); %Calculated allocation
 
%% Selecting testing scenario
 
% Setup 1
% objs = 2;                       % Number of Targets
% qualities = [0.5,0.5];          % Target qualities (q)
% preset = [-4.5,7.5; 4.5,-7.5];  % Object positions
 
% Setup 2
% objs = 4;
% qualities = 0.25*ones(1,4);
% % preset = [-4.5,7.5; 4.5,-7.5;-4.5,-7.5; 4.5,7.5];
% preset = [7.5,-4.5,;-7.5,4.5;-7.5,-4.5;7.5,4.5];
 
% Setup 3
objs = 4;
qualities = [0.1,0.2,0.3,0.4];
% preset = [-4.5,7.5; 4.5,-7.5;-4.5,-7.5; 4.5,7.5];
preset = [7.5,-4.5,;-7.5,4.5;-7.5,-4.5;7.5,4.5];
 
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
% env.showRobotIds = false;
 
%% Initialises robots and arena
robots = cell(numRobots,1);
 
%Arena dimensions
diffX = 2.125;
diffY = 1.5; 
 
%Function to initialise all robots
robots = initBots(robots,objs,diffX,diffY); 
 
% Allows for a sample robot position set to be loaded for specific tests
% save('robots.mat','robots'); 
% robots = load('robots.mat');
% robots = robots.robots;
 
%% Setting up the visualisation
%This process is slow, so can be skipped to save on processing time with
%larger problems.
 
%Extracts the robot poses
poses = extPoses(robots);
env.Poses =  poses;
 
%Draws the multi robot environment (this is an expensive operation, so not ran in loop). 
env(1:numRobots, poses, objects);
 
%Draws the robot arena bounderies.
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

%Used to enforce the bounds for the probelm space
Range = repmat((ub-lb),[FoodNumber 1]);
Lower = repmat(lb, [FoodNumber 1]);
    

%% Main ABC Algorithm Loop.
for i = 1:runs
    
    %% Initialisation of a run
    % 10 possible solutions used

%     tests = floor(FoodNumber ./ 2);

    %Randomly initialises the possible solutions (A), accounting for probelm
    %bounds
    A = (rand(FoodNumber,D) .* Range) + Lower;
      
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
    
    %Resets trail array
    trial=zeros(1,FoodNumber); %reset trial counters
    iter=1; %Resets iteration count
    
    %% Main Iteration Loop
%     sampleFits = zeros(numRobots-objs,FoodNumber); %Testing variable
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
            gTrails = 0;
        end
        
        
        %% Scout Bee Phase
        
        %Only one solution can be scouted per iteration
        ind=find(trial==max(trial));
        ind=ind(end);
 
        % If the maximum trail value is greater than the limit, reset the
        % counter and choose a new solution for that individual.
        if (trial(ind)>limit)
            trial(ind)=0;
            
            % As with initial solutions, scouted solutions are generated
            % randomly
            sol = (rand(1,D) * Range(ind)) + Lower(ind);
            sol = round(sol);
              
            % Updates the fitness values to include the new solution
            FitnessSol=beeFit(1,sol,robots,qualities,0);
            
            A(ind,:)=sol;
            fitness(ind)=FitnessSol;
        end
%         
%         if(mod(iter,Coeff) == 0)
%             sampleFits(iter/Coeff,:) = fitness;
%         end
 
        % Record all ten fitness values for plotting results.
        sampleFits(iter,:) = fitness;
        iter = iter + 1;
    end
    
    %% End of Loop Recording
    
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
    
    waitfor(0.01)
end
%%    
% Plots how the fitness for all 10 potential solutions evolved through the
% last run.
figure(3)
plot(sampleFits)
 
% Extends the final allocation set to include static robots for plotting.
sample = [1:objs,GlobalMaxes(end,:)];

% maxGMax analysis
% sample = [1:objs,GlobalMaxes(maxGMax,:)]; 
% robots = maxRobs;
 
%% Producing an example visualisation
% %Initialising main environment visualisation
figure(1)
poses = extPoses(robots);
env.Poses =  poses;
env(1:numRobots, poses, objects);
xlim(limX);   % Without this, axis resizing can slow things down
ylim(limY);  
 
%Sets object labels
for i = 1:objs
    text(objects(i,1) - 0.05,objects(i,2) - 0.3,num2str(i),'Color',[0,0,0],'FontWeight','bold');
end
 
figure(1)
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
 
%% Bar chart comparing average allocation across all runs to desired allocation
 
%Counts the total number of robots allocated to each task, then normalises
%the results
tmp = histcounts(sample,'Normalization','probability');
tmp = tmp*100;
 
%Finds the visual error between the bars (not necessarily equal to
%MAE)
visE = abs(tmp(1)-tmp(2))/2;
 
%X axis for the bar chart 
x = 1:length(tmp);
 
%Y axis for the bar chart
for i =1:length(tmp)
    y(i,1) = tmp(i);
    y(i,2) = 100*(qualities(i)/ sum(qualities));
end
 
% Showing the bar chart
figure(2);
 
tmp = bar(x,y,0.75);
 
%Sets the colour for the obtained distribution (cyan)
tmp(1).FaceColor = [0 1 1]; 
 
%Sets the colour for the obtained distribution (green)
tmp(2).FaceColor = [0 1 0]; 
 
%Sets up the legend labels for the bar chart
set(tmp, {'DisplayName'}, {'Obtained','Expected'}');
 
%Axes labels for the bar chart
xlabel('Target');
ylabel('Number of robots (%)');
ylim([0 50]); %0.7 1, 0.4 2, 0.5 3
 
% Showing the legend for the bar chart
legend('Location','northwest')
 
%% Calculation of results
 
%Calculating the maximum and average MAE
maxE = max(mae);
avgE = mean(mae);
 
%Displaying the average and maximum mae alongside total distance
%Note that mae is shown in % and distance is shown in m.
disp('   AvgE(%)   MaxE(%)   Avg Dist');
disp([100*avgE 100*maxE (mean(dists))]);
 
% Writing results to csv for validation
% out = [GlobalMaxes,mae',allDists'];
% %% Writing results to csv for validation
% writematrix((out),'Test.csv');