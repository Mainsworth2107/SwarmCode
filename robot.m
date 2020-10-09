classdef robot < handle
    
    %% Public Properties
    properties
        printAt = 100;
        
        id = [];
        pose = [];
        state = [];
        
        goal = [];
        minT = [];
        vel = [];
        %detected objects
        detObjs = [];
        %detected robots
        detRobs = [];
    end
    
    %% Public Methods
    methods
        function obj = robot(N, M)
            obj.pose = N;
            obj.id = M;
        end

        
        function obj = cycle(obj,its,detections) 
            %Prints a label for each shown iteration
            if mod(its,obj.printAt) == 0
                disp(['Iteration', num2str(its)]);
                disp([newline,newline]);
            end
            
            switch(obj.state)
                case 0
                    obj = searching(obj,its,detections);
                case 1
                    %Intermittent state that sets goal params
                    obj.prepHoming();
                    obj.state = 2;
                case 2          
                    obj.turning();
                case 3
                    obj.moving();
                case 4
                    
                otherwise
                    disp([newline, 'Read in invalid state: ', ...
                        num2str(obj.state),' Terminating...']);
            end
            if(its == 128)
                obj.state = 1;
            end
        end
        
        %Sets paremeters for turning (intermittent state)
        function obj = prepHoming(obj)
            if ~isempty(obj.detObjs)
                %min = [-1,0,0]; %Furthest detected obj  
                min = [20,0,0];
                for j = 1:length(obj.detObjs(:,1)) 
                    if (obj.detObjs(j,1) > 0) && (obj.detObjs(j,1) < min(1))
                        min = obj.detObjs(j,:);
                    end
                end
                obj.minT = min;
                
                obj.goal = ...
                [obj.pose(1) + (  min(1)*cos(min(2)) ), ...
                 obj.pose(2) + (  min(1)*sin(min(2)) ), ...
                 fixPose(min(2)), min(3)];
             
                
            else %Defaults to resting if nothing seen
                obj.state = 4;
                
            end
        end
        
        function obj = turning(obj)
            step = 1.6;
            tmp = (obj.goal(3) - obj.pose(3));
            if(abs(tmp) > pi)
                tmp = tmp + (2*pi*(tmp / abs(tmp)));                        
            end

            %tmp = fixPose(tmp + pi) - pi;
            if(abs(tmp) > (deg2rad(step)))
                obj.pose(3) = fixPose(obj.pose(3) + ...
                    (deg2rad(step) * (tmp / abs(tmp)))  );     
            else
                obj.pose(3) = obj.goal(3);
                obj.vel = bodyToWorld([0.5;0;0],obj.pose);
                obj.state = 3;
            end
        end  
        
        function obj = moving(obj)
            if distEu(obj.pose(1:2)',obj.goal(1:2)) > 0.05
                obj.pose(1:2) = obj.pose(1:2) + obj.vel(1:2)*0.02; 
            else
                obj.state = 4;
            end 
        end
        
        function obj = dispRob(obj,inA)
            switch(obj.state)
                case 0
                disp(['Robot', num2str(obj.id)]);
                if ~isempty(inA)
                    nearestLabel = inA(1,3);
                    range = inA(1,1);
                    disp(['Nearest object is of label [', num2str(nearestLabel), ']']); 
                    disp(['Distance of object: ' num2str(range)]);
                    %disp(['Detections :', mat2str(detections)])
                else
                    disp('No objects detected'); 
                end
                otherwise
            end
        end 
        
    end 
end