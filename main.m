%% Swarm robotics hello world.
%Uses The MathWorks, Inc. As a baseline.
% Rendering the env is slow. Fewer renderings drastically increase speed
flush
%% Create a multi-robot environment

numRobots = 10; %Self explanitary

env = MultiRobotEnv(numRobots); %Initalises robot envrionment
env.showTrajectory = false; %Disables live pathing
% env.robotRadius = 0.25; %Determines robot chunkiness

limX = [-14 14]; % 8
limY = [-10, 10];

A = zeros(1,numRobots); %Allocation
%% Adding Objects to environment
%objs = 10; %Number of objects

%setup 1
% objs = 2;
% qualities = [0.5,0.5];

objs = 2;
% qualities = [0.1,0.2,0.3,0.4];
qualities = [0.5 0.5];
%qualities = [100,250,300,150]; %(16) robots (2, 5, 6 ,3)


objects = zeros(objs,3);
colours = objects;

%Stores the detection data of an object.
detected = [[],[]];
%preset = [2,2;2,14;14,2;14,14]; %Must be the same size as all objects
preset = [-4.5,7.5; 4.5,-7.5];% ;-4.5,-7.5; 4.5,7.5]; %];

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

global allObjs;
allObjs = [objects,(qualities')];

%save('objs.mat','objects');

env.hasObjects = 1;
env.objectColors = colours;
env.objectMarkers = objstr;

% Hides robot IDs
env.showRobotIds = false;

%% Initalises robots
robots = cell(numRobots,1);

% preset = [1.5,1;4,2;3.5,4;6,4];
%%
%rad = 4;
% Whole Arena
diffX = 21.25;
diffY = 15;

% Out to the objects
% diffX = 15;
% diffY = 9;

robots = initBots(robots,objs,diffX,diffY);

dTheta = pi/64;
ranges = cell(1,numRobots);

%angles = linspace(-pi,pi,100);

%% Setting up the visulisation

%Initalising main envrionment visu
poses = extPoses(robots);
env.Poses =  poses;
env(1:numRobots, poses, objects);
xlim(limX);   % Without this, axis resizing can slow things down
ylim(limY);  

% Sets ojbect labels
for i = 1:objs
    text(objects(i,1) - 0.05,objects(i,2) - 0.3,num2str(i),'Color',[0,0,0],'FontWeight','bold');
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runs = 50;
mae = zeros(1,runs);
dists = mae;
for i = 1:runs

    counts = zeros(1,objs);
    for j = 1:numRobots
        idx = (j + (numRobots*(i-1)));
        if j <= objs
            A(i,j) = j;
            counts(A(i,j)) = counts(A(i,j)) + 1;
        else
            [P,Q] = newFit(robots{j}.pose);
            % MAE does not consider already allocated robots.
            A(i,j) = P;
            dists(i) = dists(i) + Q;
            counts(A(i,j)) = counts(A(i,j)) + 1;
        end
    end
    counts = counts ./ (sum(counts));
    mae(i) = (1/objs) * sum(abs(qualities - counts)); 
    mum(i) = mae(i);
    if counts(1) < counts(2)
        mum(i) = mum(i)*-1;
    end
    robots = initBots(robots,objs,diffX,diffY);
end
%%

sample = A(ceil(rand*height(A)),:);
poses = extPoses(robots);

for i = objs:numRobots
    line([robots{i}.pose(1),objects(sample(i),1)],...
         [robots{i}.pose(2),objects(sample(i),2)],...
         'color','black','LineWidth',1);
end
env.Poses =  poses;
env(1:numRobots, poses, objects);


%% Bar Chart

tmp = histcounts(A,'Normalization','probability');
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
%% Error Max and average

maxE = max(mae);
avgE = mean(mae);
visE = abs(visE(1)-visE(2))/2;

%%
format short
disp('   AvgE(%)   MaxE(%)   Avg Dist');
% disp('    AvgE      MaxE');
disp([100*avgE 100*maxE (mean(dists)/10)]);

out = [floor(A),mae',dists'];
%%
writematrix((out),'Test.csv');

