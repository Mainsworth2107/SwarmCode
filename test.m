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
%%
poses = extPoses(robots);
env.Poses =  poses;
dTheta = pi/64;
ranges = cell(1,numRobots);

%angles = linspace(-pi,pi,100);

%% Setting up the visulisation

%Initalising main envrionment visu
env(1:numRobots, poses, objects);
xlim([0 8]);   % Without this, axis resizing can slow things down
ylim([0 8]);  

% Sets ojbect labels
for i = 1:objs
    text(objects(i,1) - 0.05,objects(i,2) - 0.3,num2str(i),'Color',[0,0,0],'FontWeight','bold');
end

%% Running the simulation
%its 1 - 128 are the turn, it 129 initalises the turn
for idx = 1:129 %Equiv to a single 360 turn (at turn margin 1 / 64)
    % Get the current time step's ranges
    %scans = lidar();
    %ranges{lidar.robotIdx} = scans;
       
    % Loop that each robot runs
    for j = 1:numRobots
        %Called in main to allow singular detector definiton (more
        %efficent)
        detections = detector(robots{j}.pose,objects);
        %Calling the main robot loop
        robots{j}.cycle(idx,detections);
    end    
    
    % Update the environment and poses after control loop
    poses = extPoses(robots);
    env(1:numRobots, poses, objects);
    xlim([0 8]);   % Without this, axis resizing can slow things down
    ylim([0 8]); 
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Delay between 'search' and 'react' phases for more obvious event flow
disp('Ready to move?')
waitforbuttonpress

%% Moving to an object
moved = 1;
its = 0;
tmp = 1;
action = zeros(1,numRobots);

% toPlot = [];

%%
step = 1.6;
moved = 1;
while(moved)
    moved = 0;
    for i = 1:numRobots
        if ~isempty(robots{i}.detObjs)
            switch(action(i))
                case 0             
        %             if i == 1
        %                 toPlot(1,tmp) = tmp;
        %                 toPlot(2,tmp) = (distEu(robots{i}.pose(3),robots{i}.goal(3)) > 0.05);
        %                 tmp = tmp + 1;
        %             end
                    tmp = (robots{i}.goal(3) - robots{i}.pose(3));
                    if(abs(tmp) > pi)
                        tmp = tmp + (2*pi*(tmp / abs(tmp)));                        
                    end
                    
                    %tmp = fixPose(tmp + pi) - pi;
                    if(abs(tmp) > (deg2rad(step)))
                        robots{i}.pose(3) = fixPose(robots{i}.pose(3) + ...
                            (deg2rad(step) * (tmp / abs(tmp)))  );     
                    else
                        robots{i}.pose(3) = robots{i}.goal(3);
                        robots{i}.vel = bodyToWorld([0.5;0;0],robots{i}.pose);
                        action(i) = 1;
                    end
                    
                case 1                     
                    if distEu(robots{i}.pose(1:2)',robots{i}.goal(1:2)) > 0.05
                        robots{i}.pose(1:2) = robots{i}.pose(1:2) + robots{i}.vel(1:2)*0.02; 
                    else
                        action(i) = 2;
                    end 
%                     if (mod(its,10) == 0)
%                        waitfor(0.01)
%                     end
            end   
        else
            action(i) = 2;
        end
    end
    
    for i = 1:numRobots
        if ~(action(i)  == 2)
            moved = 1;
        end
    end
    %Renders robots every third iteratiom
    its = its + 1;
    if(its > 10000)
        break
    end
    if(mod(its,3) == 0)
        poses = extPoses(robots);
        env(1:numRobots, poses, objects);
    end  
    pause(0.015)
end
% 
% figure(2);
% plot(toPlot(1,:),toPlot(2,:));