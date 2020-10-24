function X = initBots(robots,objs,diffX,diffY)
    global allObjs
    for i = 1:size(robots)
%     angle = rand*2*pi;
%     robots{i} = robot([preset(i,:),(rand*pi)], i);
%     robots{i} = robot([rand*rad*cos(angle);rand*rad*sin(angle);rand*pi],i);
    robots{i} = robot((rand(3,1).*[diffX;diffY;pi]) - ...
        (0.5 *[diffX;diffY;0]),i);
%     robots{i} = robot((normrnd(0,0.5,3,1).*[diffX;diffY;pi]) - ...
%         (0.5 *[diffX;diffY;0]),i);
    end
    for i = 1:objs
        robots{i}.pose(1:2) = allObjs(i,(1:2));
    end
    X = robots;
end