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



    %Loop that each robot runs
%     for j = 1:numRobots
%         %Gets detections from position (so not actually the robot)
%         detections = detector(poses(:,j),objects);
%         %records a detection
%         if ~isempty(detections)
%             its = size(detections);
%             %Looks at all currently detected objects
%                     
%             for k = 1:its(1)
%                 currentObj = detections(k,3); % Current object label
%                 tmp = size(detected);
%                 pos = tmp(1) + 1;
%                 if(tmp(1) < j)
%                     detected(j,1,:) = detections(k,:);
%                     detected(j,1,2) = detected(j,1,2) + poses(3,j);
%                 else
%                     if ~isIn(currentObj,detected(j,:,3))
%                         detected(j,pos,:) = detections(k,:);
%                         detected(j,pos,2) = detected(j,pos,2) ...
%                             + poses(3,j);
%                     end
%                 end
% %                 %If first encounter
% %                 if(detected(currentObj,1) < 0)
% %                     %Saves distance and angle(normalised)
% %                     detected(currentObj,:) = detections(k,:);
% %                     detected(currentObj,2) = detected(currentObj,2) + poses(3,j);
% %                 end
%             end        
%         end
%          
%         %Prints every tenth iteration
%         if mod(idx,10) == 0
%             disp(['Robot', num2str(j)]);
%             if ~isempty(detections)
%                 nearestLabel = detections(1,3);
%                 range = detections(1,1);
%                 disp(['Nearest object is of label [', num2str(nearestLabel), ']']); 
%                 disp(['Distance of object: ' num2str(range)]);
%                 %disp(['Detections :', mat2str(detections)])
%             else
%                 disp('No objects detected'); 
%             end
%         end 
%         
%         %Turns the robots
%         poses(3,j) = poses(3,j) + dTheta; %Index is the robot number 3 is the angle
%         %Wraps the angle to always be in the range -pi -> pi 
%         if abs(poses(3,j)) > (2 * pi)
%             poses(3,j) = poses(3,j) - (2 * pi * (poses(3,j) / abs(poses(3,j))));
%         end  
%         
%         %waitfor(r); % Delays next loop (remove to speed up but make output less readable)   
%     end