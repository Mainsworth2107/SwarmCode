%% Old misc code, may be useful later.

%Loads in OGM template for encolse environments
%load exampleMap
%env.mapName = 'map';

% %% Creates a lidar sensor (only works with an OGM)
% index = 1;
% lidar = MultiRobotLidarSensor;
% lidar.robotIdx = index;
% lidar.scanAngles = linspace(-pi/4,pi/4,12);
% lidar.maxRange = 4;
% attachLidarSensor(env,lidar);

    %env(1:numRobots, poses, ranges, objects) (for including a lidar ontop
    %of object detector)
    
% %% Moving to an object
% % Note: There is the potential for one robot to block the path of 
% % another, making it so the object is not reached.
% for i = 1:numRobots
%     detections = detector(poses(:,i),objects); 
%     vel = bodyToWorld([0.5;0;0],poses(:,i));
%     while  ~isempty(detections) && detections(1,1) > 0.01 %assumes closest is always goal (not the case)
%         detections = detector(poses(:,i),objects);
%         j = j +1;
%         if(j > 1000)
%             break
%         end
%         poses(:,i) = poses(:,i) + vel*0.05;
%         env(1:numRobots, poses, objects);
%         waitfor(0.05)
%     end
% end