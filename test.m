%% Swarm robotics hello world.
%Uses The MathWorks, Inc. As a baseline.
% Rendering the env is slow. Fewer renderings drastically increase speed
flush
%% Create a multi-robot environment

numRobots = 4; %Self explanitary

env = MultiRobotEnv(numRobots); %Initalises robot envrionment
env.showTrajectory = false; %Disables live pathing
env.robotRadius = 0.25; %Determines robot chunkiness

limX = [0 8]; % 8
limY = [0 8];

A = zeros(1,numRobots); %Allocation

%% Adding detectors
% Adding Object Detector
% Cleverly, only one OD is needed, it is simmply moved to the exact pos /
% angle needed for each robot
detector = ObjectDetector;
detector.fieldOfView = pi/4;
detector.maxRange = 3;
%attachObjectDetector(env,1,detector);

% Adding robot detector
%Altough initalised with a specific attatchment, can also be reattatched.
robDet = RobotDetector(env,3);
% robDet.fieldOfView = pi/4; %Must be the same os the obj detector
% robDet.maxRange = 3;
robDet.fieldOfView = 2*pi; %Must be the same os the obj detector
robDet.maxRange = 3;
robDet.maxDetections = numRobots;
%attachRobotDetector(env,robDet);
%% Adding Objects to environment
mins = [0,0];
maxes = [6,6];

%objs = 10; %Number of objects
objs = 4;
% steps = 25; %Placement precisions (on both axes, so sqare for total pos num)

objects = zeros(objs,3);
colours = objects;

%Stores the detection data of an object.
detected = [[],[]];
preset = [5,5;5,1;6,2;7.5,5];

objstr = 'o';
for i = 1:objs
   objects(i,3) = i;
   % Retunrns the x and y corrds of the object.
   %First eq returns the step (it from 0). The +1 is to stop near edge spwans
   %Second eq returns the scale, the +2 stops overflow and far edge spawns
   %thrid corrects position
%    for j = 1:2
%        objects(i,j) = ((floor(steps*rand) + 1) * ...
%            ((maxes(j) - mins(j)) / (steps + 2))) ...
%            + mins(j);
%    end
    objects(i,1:2) = preset(i,:);
   
   % Sets the necessary labels to draw all objects as circles
   if( i > 1)
       objstr = strcat(objstr,'o');
   end  
   %Makes all objects red
   colours(i,1) = 1;
   colours(i,2:3) = 0;
end
%sets the necessary flags in the evrinment class to reflect the object
%definitions
%save('objs.mat','objects');



env.hasObjects = 1;
env.objectColors = colours;
env.objectMarkers = objstr;

%% Initalises robots
robots = cell(numRobots,1);

preset = [1.5,1;4,2;3.5,4;6,4];
%%
for i = 1:numRobots
    %robots{i} = robot(4*(rand(3,1).*[1;1;pi] - [0.5;0.5;0]) + [3;3;0], i);
    robots{i} = robot([preset(i,:),(rand*pi)], i);
    robots{i}.state = 0;
end

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

%% Running the simulation
%its 1 - 128 are the turn, it 129 initalises the turn its 129 + are moving
its = 1; %total iteration count
%step = 1.6;
%%
global others;
others = robots;
moved = 1; %end state recognition

while(moved)
    moved = 0;
    % Get the current time step's ranges
    %scans = lidar();
    %ranges{lidar.robotIdx} = scans;
       
    % Loop that each robot runs
    for i = 1:numRobots
        %Called in main to allow singular detector definiton (more
        %efficent)
        detections = detector(robots{i}.pose,objects);
        %Calling the main robot loop
%         others = robots;
        robots{i}.cycle(its,detections);
    end    
    
    for i = 1:numRobots
        if ~(robots{i}.state  == 4) % if resting
            moved = 1;
        end
    end
    
    % Update the environment and poses after control loop
    if(mod(its,3) == 0)
        poses = extPoses(robots);
        env(1:numRobots, poses, objects);
        xlim(limX);   % Without this, axis resizing can slow things down
        ylim(limY); 
    end
    
    its = its + 1;
    if(its > 128)
        break
    end
    
    pause(0.015)
end
%% Allocation

% Runs the allocation (called on it 129)
while(its < 131)
    others = robots;
    for i = 1:numRobots
        robots{i}.cycle(its,[]);
    end
    its = its +1;
end
%%
% clc
% % Random allocation sist, start f=thinking of comms here
% others = robots;
% for i = 1:numRobots
%     %A(i) = robots{i}.A;
%     robots{i}.setVars();
%     [robots{i}.A, getFit(floor(robots{i}.A))]
% end
%A;
%%
%Limited range comms
% others = tmp;
% robDet.robotIdx = toTest;
% step = robDet.step;
% 
% % This is the IDs of the seen robots
% 
% %Single hop ranged comms
% if ~isempty(step) %If detected robots is empty
%     
%     %Single hop ranged comms
%     idxs = step(:,3);
% 
%     for i = 1:length(idxs) 
%         others{i} = robots{idxs(i)};
%     end
%     robots{toTest}.commsTest();
% else
%     robots{toTest}.knownObjs = robots{toTest}.detObjs;
% end

%A
