%% Swarm robotics hello world.
%Uses The MathWorks, Inc. As a baseline.
% Renderint the env is slow. Fewr renderings drastically increase speed

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
attachObjectDetector(env,1,detector);

sampleTime = 0.001; %Sample time (LIMTIS SIMULATION SPEED)
%r = rateControl(1/sampleTime); %Sample wait period
%r = 0.01;
r = 0;
%% Adding Objects to environment
mins = [0,0];
maxes = [12,12];

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
env.hasObjects = 1;
env.objectColors = colours;
env.objectMarkers = objstr;


%% Initalises robot positions
poses = 4*(rand(3,numRobots).*[1;1;pi] - [0.5;0.5;0]) + [9;9;0];
%poses = 4*(rand(3,1).*[1;1;pi] - [0.5;0.5;0]) + [9;9;0];

%poses = [4,7;6,6;pi/2,pi/2];
%poses = [4,7;6,6;0,0];
%poses = [7;7;0];
env.Poses =  poses;
dTheta = pi/64;
ranges = cell(1,numRobots);

%angles = linspace(-pi,pi,100);

%% Setting up the visulisation

%Initalising main envrionment visu
env(1:numRobots, poses, objects);
xlim([0 16]);   
ylim([0 16]); 

% Sets ojbect labels
for i = 1:objs
    text(objects(i,1) - 0.05,objects(i,2) - 0.3,num2str(i),'Color',[0,0,0],'FontWeight','bold');
end

%% Running the simulation
for idx = 1:128 %Equiv to a single 360 turn (at turn margin 1 / 64)
    % Get the current time step's ranges
    %scans = lidar();
    %ranges{lidar.robotIdx} = scans;
    
    %Prints a label for each shown iteration
    if mod(idx,10) == 0
        disp(['Iteration', num2str(idx)]);
        disp([newline,newline]);
    end
    
    %Loop that each robot runs
    for j = 1:numRobots
        %Gets detections from position (so not actually the robot)
        detections = detector(poses(:,j),objects);
        %records a detection
        if ~isempty(detections)
            its = size(detections);
            %Looks at all currently detected objects
            
            %Just doing R1 RN
            
            for k = 1:its(1)
                currentObj = detections(k,3); % Current object label
                tmp = size(detected);
                pos = tmp(1) + 1;
                if(tmp(1) < j)
                    detected(j,1,:) = detections(k,:);
                    detected(j,1,2) = detected(j,1,2) + poses(3,j);
                else
                    if ~isIn(currentObj,detected(j,:,3))
                        detected(j,pos,:) = detections(k,:);
                        detected(j,pos,2) = detected(j,pos,2) ...
                            + poses(3,j);
                    end
                end
%                 %If first encounter
%                 if(detected(currentObj,1) < 0)
%                     %Saves distance and angle(normalised)
%                     detected(currentObj,:) = detections(k,:);
%                     detected(currentObj,2) = detected(currentObj,2) + poses(3,j);
%                 end
            end
            
            
            %Turns the robots (note: this means that the inital pose is not
            %shown)
        end
         
        %Prints every tenth iteration
        if mod(idx,10) == 0
            disp(['Robot', num2str(j)]);
            if ~isempty(detections)
                nearestLabel = detections(1,3);
                range = detections(1,1);
                disp(['Nearest object is of label [', num2str(nearestLabel), ']']); 
                disp(['Distance of object: ' num2str(range)]);
                %disp(['Detections :', mat2str(detections)])
            else
                disp('No objects detected'); 
            end
        end 
        
        %Turns the robots
        poses(3,j) = poses(3,j) + dTheta; %Index is the robot number 3 is the angle
        %Wraps the angle to always be in the range -pi -> pi 
        if abs(poses(3,j)) > (2 * pi)
            poses(3,j) = poses(3,j) - (2 * pi * (poses(3,j) / abs(poses(3,j))));
        end  
        
        %waitfor(r); % Delays next loop (remove to speed up but make output less readable)   
    end
    
    %% Update the environment and poses after control loop
    % Updates plot
    env(1:numRobots, poses, objects);
    xlim([0 16]);   % Without this, axis resizing can slow things down
    ylim([0 16]); 
    % Breakpoint statement for testing
    if(mod(idx, 10) == 0)
        %waitfor(0.01) %Breakpoint setting line
    end
    %waitfor(r); % Delays next loop (remove to speed up but make output less readable)
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Delay between 'search' and 'react' phases for more obvious event flow
disp('Ready to move?')
waitforbuttonpress
%%
%Pos values: 1: desx, 2:desy, 3: destheta
%Stores the vector values that correspond to the direstion of the nearest
%object
vel = zeros(3, numRobots);
%stores the coords of the nerest object
goal = zeros(3, numRobots);
%% Turning towards an object
for i = 1:numRobots
    min = [-1,0,0]; %Furthest detected obj       
    for j = 1:length(detected(i,:,1))
        if (detected(i,j,1) > 0) && (detected(i,j,1) > min(1))
            %The normal : notation spits out an inconvient 3x1x1 matrix
            %here
            min = [detected(i,j,1),detected(i,j,2),detected(i,j,3)];
        end
    end
    
    poses(3,i) = min(2);
   
    env(1:numRobots, poses, objects);
    % This is used in the movemnt section, saves loop definition
    
    goal(1:2,i) = objects(min(3),1:2); 
    vel(:,i) = bodyToWorld([0.5;0;0],poses(:,i));
end
 
%% Moving to an object

moved = 1;
its = 0;
while(moved)
    moved = 0;
    for i = 1:numRobots    
        if distEu(poses(1:2,i),goal(1:2,i)) > 0.05
            poses(:,i) = poses(:,i) + vel(:,i)*0.05;
            if(mod(its,3) == 0)
                env(1:numRobots, poses, objects);
            end
            moved = 1;
        end
        its = its + 1;
        if(its > 10000)
            break
        end
    end
end