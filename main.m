%% Distributed Bees Algorithm
% Written by Matthew Ainsworth
% Last modified: 12/11/2020

%% Acknowledgements
 
% Based on: Distributed Bees Algorithm for Task Allocation in Swarm of Robots
% Jevtić et al, 2012
 
% Uses The Mobile Robotics Simulation Toolbox (MRST) from: The MathWorks, Inc.
% Note: A small modification was made to the MRST code to draw the objects
% in the environment as slightly larger than the default size.
 
%N represents the number of robots (in comments)
%M represents the number of objects (in comments)
 
%% Create a multi-robot environment
flush
numRobots = 100; %Initialises the number of robots.
 
env = MultiRobotEnv(numRobots); %Initialises robot envrionment (MRST)
env.showTrajectory = false; %Disables robot pathing 
 
% Limits of visualisation axes
limX = [-1, 1];
limY = [-1.4 1.4]; 
 
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
 
% Setup 3
objs = 4;
qualities = [0.1,0.2,0.3,0.4];
preset = [-4.5,7.5; 4.5,-7.5;-4.5,-7.5; 4.5,7.5];
 
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
diffX = 1.5;
diffY = 2.125;
 
%Function to initialise all robots
robots = initBots(robots,objs,diffX,diffY); 
 
% Allows for a sample robot position set to be loaded for specific tests
% save('robots.mat','robots'); 
% robots = load('robots.mat');
% robots = robots.robots;
 
%% Setting up the visualisation
 
%Extracts the robot poses
poses = extPoses(robots);
env.Poses =  poses;
 
%Draws the multi robot environment (this is an expensive operation, so not ran in loop). 
env(1:numRobots, poses, objects);
 
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
 
%% Running the algorithm
runs = 50; %Total scenarios (robot position sets) to be tested
 
%Initialises output variables
mae = zeros(1,runs); % Mean absolute error.
times = mae;
dists = mae;
for i = 1:runs
    tic;
    counts = zeros(1,objs); %Counts of each allocation for calculating MAE
    
    % For Each robot
    for j = 1:numRobots
        idx = (j + (numRobots*(i-1)));
        
        if j <= objs % The first M robots will be allocated to their respective objects
            A(i,j) = j;
            counts(A(i,j)) = counts(A(i,j)) + 1;
        else
            
            % Finds the allocation using the DBA fitness function, also
            % returning distance
            [P,Q] = DBAFit(robots{j}.pose); 
            A(i,j) = P;
            
            %The distance does not need to consider the robots at objects
            %As they are always 0 units from their respective goals.
            dists(i) = dists(i) + Q;
            
            % Adding 1 on each count ensure the robots on objects are
            % considered
            counts(A(i,j)) = counts(A(i,j)) + 1;
        end
    times(i) = toc;
    end
    % Normalises the robot counts
    counts = counts ./ (sum(counts));
    
    %Finds the mean absolute error between the actual robot distribution and
    %the expected robot distribution
    mae(i) = (1/objs) * sum(abs(qualities - counts)); 
    
    %If not the final run, initialises a new robot position set (to test for consistency)
    if (i < runs)
        robots = initBots(robots,objs,diffX,diffY);
    end
end
%% Producing an example visualisation
%Example allocation chosen as last in set.
sample = A(end,:); 
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
tmp = histcounts(A,'Normalization','probability');
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

O = zeros(runs,objs);
for i = 1:height(A)
    O(i,:) = histcounts(A(i,:));
end
% disp('Average time per run (ms)');
% times = 1000*times;
% disp(mean(times));
% 
% disp('Average time per robot (μs)');
% disp(1e3*(mean(times)/numRobots))

% Writing results to csv for validation
out = [floor(O),mae']; %,dists',times'];
% out = [floor(A(1,:)),mae(1)',dists(1)'];
writematrix((out),'Test.csv');