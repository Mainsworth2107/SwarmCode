%% Swarm robotics hello world.
%Uses The MathWorks, Inc. As a baseline.
% Rendering the env is slow. Fewer renderings drastically increase speed
flush
%% Create a multi-robot environment

numRobots = 4; %Self explanitary

env = MultiRobotEnv(numRobots); %Initalises robot envrionment
env.showTrajectory = false; %Disables live pathing
env.robotRadius = 0.25; %Determines robot chunkiness

%% Adding Object Detector
% Cleverly, only one OD is needed, it is simmply moved to the exact pos /
% angle needed for each robot
detector = ObjectDetector;
detector.fieldOfView = pi/4;
%attachObjectDetector(env,1,detector);

%r = 0.01;

%% Adding Objects to environment
mins = [0,0];
maxes = [6,6];

objs = 10; %Number of objects
steps = 25; %Placement precisions (on both axes, so sqare for total pos num)

objects = zeros(objs,3);
colours = objects;

%Stores the detection data of an object.
detected = [[],[]];

objstr = 'o';
for i = 1:objs
   objects(i,3) = i;
   % Retunrns the x and y corrds of the object.
   %First eq returns the step (it from 0). The +1 is to stop near edge spwans
   %Second eq returns the scale, the +2 stops overflow and far edge spawns
   %thrid corrects position
   for j = 1:2
       objects(i,j) = ((floor(steps*rand) + 1) * ...
           ((maxes(j) - mins(j)) / (steps + 2))) ...
           + mins(j);
   end
   
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
save('objs.mat','objects');
env.hasObjects = 1;
env.objectColors = colours;
env.objectMarkers = objstr;

detector = ObjectDetector;
detector.fieldOfView = pi/4;

%% Initalises robots
robots = cell(numRobots);

for i = 1:numRobots
    robots{i} = robot(4*(rand(3,1).*[1;1;pi] - [0.5;0.5;0]) + [3;3;0], i);
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
xlim([0 8]);   % Without this, axis resizing can slow things down
ylim([0 8]);  

% Sets ojbect labels
for i = 1:objs
    text(objects(i,1) - 0.05,objects(i,2) - 0.3,num2str(i),'Color',[0,0,0],'FontWeight','bold');
end

%% Running the simulation
%its 1 - 128 are the turn, it 129 initalises the turn its 129 + are moving
its = 1; %total iteration count
%step = 1.6;
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
        robots{i}.cycle(its,detections);
    end    
    
    for i = 1:numRobots
        if ~(robots{i}.state  == 4) % if resting
            moved = 1;
        end
    end
    
    its = its + 1;
    if(its == 128)
        waitforbuttonpress();
    end
    if(its > 10000)
        break
    end
    % Update the environment and poses after control loop
    if(mod(its,3) == 0)
        poses = extPoses(robots);
        env(1:numRobots, poses, objects);
        xlim([0 8]);   % Without this, axis resizing can slow things down
        ylim([0 8]); 
    end
    pause(0.015)
end
