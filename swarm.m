%% EXAMPLE: Multi-Robot Swarm Behavior
% Copyright 2018 The MathWorks, Inc.
clc
clear
%% Create a multi-robot environment
numRobots = 50;
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.15;
env.showTrajectory = false;

% %% Adding Objects (hopefully)
% mins = [-10,-10];
% maxes = [10,10];
% 
% objs = 10;
% steps = 10;
% 
% objects = zeros(objs,3);
% colours = objects;
% objstr = 'o';
% for i = 1:objs
%    objects(i,3) = i;
%    % Retunrns the x and y corrds of the object.
%    %First eq returns the step (it from 0). The +1 is to stop near edge spwans
%    %Second eq returns the scale, the +2 stops overflow and far edge spawns
%    %thrid corrects position
%    for j = 1:2
%        objects(i,j) = ((floor(steps*rand) + 1) * ...
%            ((maxes(j) - mins(j)) / (steps + 2))) ...
%            + mins(j);
%    end
%    
%    if( i > 1)
%        objstr = strcat(objstr,'o');
%    end  
%    colours(i,1) = 1;
%    colours(i,2:3) = 0;
% %    for j = 1:3
% %     colours(i,j) = (floor(rand * 200) + 25) / 256; 
% %    end
% end
% 
% env.hasObjects = 1;
% env.objectColors = colours;
% env.objectMarkers = objstr;


%% Create robot detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = 5;
    detector.fieldOfView = pi/2;
    detectors{rIdx} = detector;
end
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals
%% Initialization
% Number of robot teams
numTeams = 5;  
env.robotColors = repmat(hsv(numTeams),[ceil(numRobots/numTeams) 1]);
env.robotColors = env.robotColors(1:numRobots,:); % Truncate colors in case there are unequal teams
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:25;        % Time array                
% Initialize poses randomly, and add bias to each "team"
poses = [10*(rand(2,numRobots) - 0.5); ...
         pi*rand(1,numRobots)];
angleBias = 2*pi*(1:numRobots)/numTeams;
poses(1:2,:) = poses(1:2,:) + 2.5*[sin(angleBias);cos(angleBias)];
%% Simulation loop
vel = zeros(3,numRobots);
for idx = 2:numel(tVec)
    
    % Update the environment
    env(1:numRobots, poses);
    %env(1:numRobots, poses, objects);
    
    xlim([-16 16]);   % Without this, axis resizing can slow things down
    ylim([-16 16]); 
    
    % Read the sensor and execute the controller for each robot
    for rIdx = 1:numRobots
       detections = step(detectors{rIdx}); 
       vel(:,rIdx) = swarmTeamController(poses,rIdx,detections,numTeams);
    end
    
    % Discrete integration of pose
    poses = poses + vel*sampleTime;
end
%% Helper function: Robot Controller Logic
function vel = swarmTeamController(poses,rIdx,detections,numTeams)
    
    % Unpack the robot's pose and team index
    pose = poses(:,rIdx);
    teamIdx = mod(rIdx,numTeams);
    % If there are no detections, turn in place
    v = 0;
    w = 2;
    
    % Else, turn towards the average of detected robots on the same team
    if ~isempty(detections)
        validInds = find(mod(detections(:,3),numTeams) == teamIdx);
        if ~isempty(validInds)
            
            % Take the average range and angle
            range = mean(detections(validInds,1));
            angle = mean(detections(validInds,2));
            
            % Move linearly to maintain a range to the nearest robot
            if range > 0.6
                %v = 0.5;
                v = 0.4;
            elseif range < 0.4
                %v = -0.5;
                v = -0.2;
            end
            
            % Turn to maintain a heading to the nearest robot
            if angle > pi/12
                %w = 2;
                w = 0.2;
            elseif angle < -pi/12
                w = -0.2;
            else
                w = 0;
            end
   
            
        end
        
    end
    % Convert to global velocity
    vel = bodyToWorld([v;0;w],pose);
end