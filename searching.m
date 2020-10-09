%% Robot searching behaviour loop.

%Called for each robot in the searching sate by the robot class.
%Defined outdside of the class for eaiser editing and modification.

function obj = searching(obj,its,detections)
    %Initalises object detector and loads in objects           
    if ~isempty(detections)
        its = size(detections);
        %Looks at all currently detected objects
        for i = 1:its(1)
            currentObj = detections(i,3); % Current object label
            tmp = size(obj.detObjs);
            pos = tmp(1) + 1;
            if(tmp(1) < 1)
                obj.detObjs(1,:) = detections(i,:);
                obj.detObjs(1,2) = detections(i,2) + obj.pose(3);
            else
                if ~isIn(currentObj, obj.detObjs(:,3))
                    obj.detObjs(pos,:) = detections(i,:);
                    %Records angle from global zero (+ x axis)
                    obj.detObjs(pos,2) = fixPose(detections(i,2) ...
                        + obj.pose(3));
                end
            end
        end
    end
    %Turns the robot
    %pi / 64 is step
    obj.pose(3) = obj.pose(3) + (pi/64); %Index is the robot number 3 is the angle
    %Wraps the angle to always be in the range -pi -> pi 
    if abs(obj.pose(3)) > (2 * pi)
        obj.pose(3) = fixPose(obj.pose(3));
    end 

    %Prints every tenth iteration
    if mod(its,obj.printAt) == 0
        %obj.dispRob(detections);
    end  
end