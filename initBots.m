%% Refernces
%All refernces given in main.m
%
%% Function which randomly initialises the positions of the robots.
 
function X = initBots(robots,objs,diffX,diffY)
 
    global allObjs
    for i = 1:size(robots)
        %Each robot is randomly placed within the bounds of the arena
        %according to a uniform distribution.
        
        robots{i} = robot((rand(3,1).*[diffX;diffY;pi]) - ...
            (0.5 *[diffX;diffY;0]),i);
    
        %Other potential placement methods include within a circle, preset
        %position allocation and normally distributed robots around (0,0)
%     robots{i} = robot([preset(i,:),(rand*pi)], i);
%     angle = rand*2*pi;
 
%     robots{i} = robot([rand*rad*cos(angle);rand*rad*sin(angle);rand*pi],i);
%     robots{i} = robot((normrnd(0,0.5,3,1).*[diffX;diffY;pi]) - ...
%         (0.5 *[diffX;diffY;0]),i);
    end
    
    %Sets the first M robot's positions as equal to those of their
    %corresponding objects
    for i = 1:objs
        robots{i}.pose(1:2) = allObjs(i,(1:2));
    end
    X = robots;
end