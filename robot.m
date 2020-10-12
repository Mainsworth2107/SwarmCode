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
        knownObjs = [];
        
        %Allocation stuff
        A = [];
        AGbest = [];
    end
    properties(Access = private)
        %Allocation stuff
        limit = 500;
    end
    
    %% Public Methods
    methods
        %Initaliser
        function obj = robot(N, M)
            obj.pose = N;
            obj.id = M;
        end
        
        % Gets all known objects (comms example / basis)
        function getObjs(obj) 
            global others;
            
            tmp = cell(1);
            j = 0;
                   
            %Dont want a robot to call its self
            for i = 1:length(others)
                if ~(others{i}.id == obj.id)
                    j = j +1;
                    tmp{j} = others{i};
                end
            end         
                                               
            if ~isempty(obj.detObjs)
                %obj.knownObjs = obj.detObjs(:,3);
                obj.knownObjs = obj.detObjs;
            end
            
            %For each known robot
            for i = 1:length(tmp) 
                
                %For each detected object for that robot
                for j = 1:height(tmp{i}.detObjs)
                    % If known OBJS, 
                    if ~isempty(obj.knownObjs) 
%                         if ~( isIn(tmp{i}.detObjs(j,3), obj.knownObjs(:)) )
%                             obj.knownObjs = [obj.knownObjs;tmp{i}.detObjs(j,3)];
%                         end
                        if ~( isIn(tmp{i}.detObjs(j,3), obj.knownObjs(:,3)) )
                            obj.knownObjs = [obj.knownObjs; ...
                            obj.objPos( tmp{i}.pose(:) , tmp{i}.detObjs(j,:) )];
                        end
                        
                    else %If no known OBJS, then port all friends det objs
                        %obj.knownObjs = [obj.knownObjs;tmp{i}.detObjs(j,3)];
                            obj.knownObjs = [obj.knownObjs; ...
                            obj.objPos( tmp{i}.pose(:) , tmp{i}.detObjs(j,:) )];
                    end    
                end
                
            end
        end
        
        %Sets all inital values for allocation
        function setVars(obj)
            global others
            obj.getObjs();
            %Allocation is real but interpreted as 
            % initlise PB here
            obj.A =  rand(1,(length(others))) * (length(obj.knownObjs)) + 1; 
            obj.AGbest = floor(obj.A);
        end
        
%         %Runs ONE iteration of the PSO algorithm
%         function psoA(obj)
%             
%         end
        
        function X = objPos(obj,POS, min) %Distance and anlge of new object
            %Finds actual position
            tmp = zeros(1,3);
            tmp = [POS(1) + (  min(1)*cos(min(2)) ), ...
            POS(2) + (  min(1)*sin(min(2)) )];
        
            %Finds distance and angle
            memes = tmp(1:2) - obj.pose(1:2);
            tmp(3) = fixPose(atan2(memes(2),memes(1)));
            X = [distEu(obj.pose(1:2),tmp(1:2)),tmp(3),min(3)];
        end
        
        
        function obj = cycle(obj,its,detections) 
            
            %Prints a label for each shown iteration
%             if mod(its,obj.printAt) == 0
%                 disp(['Iteration', num2str(its)]);
%                 disp([newline,newline]);
%             end

            %FSM definition (currently no state looping)
            switch(obj.state)
                case 0 %Inital turn
                    % state outsourced for efficiency
                    obj = searching(obj,its,detections);
                    
                case 1 %Intermittent state that sets goal params                  
                    greedyA(obj);
                    
                case 2 %Turning to goal
                    line([obj.pose(1),obj.goal(1)]...
                        ,[obj.pose(2),obj.goal(2)]...
                        ,'color','black','LineWidth',3);
                    obj.state = 4;
                    %obj.turning();
                    
                case 3 %Moving too goal
                    obj.moving();
                case 4 %Resting state (currently does nothing)
                    
                otherwise
                    disp([newline, 'Read in invalid state: ', ...
                        num2str(obj.state),' Terminating...']);
            end
            
            % State 0 and 1 are concurrent (non varient behaviour)
            if(its == 128)
                obj.state = 1;
                obj.setVars();
            end
        end
          
        
        function obj = turning(obj)
            step = 1.6; % turning speed (deg / cycle)
            tmp = (obj.goal(3) - obj.pose(3));
            
            if(abs(tmp) > pi)
                tmp = tmp - (2*pi*(tmp / abs(tmp)));                        
            end
            
            % Turn until within one turn. Then correct angle is snapped
            % (ideal conditions)
            if(abs(tmp) > (deg2rad(step)))
                obj.pose(3) = fixPose(obj.pose(3) + ...
                    (deg2rad(step) * (tmp / abs(tmp)))  );     
            else
                obj.pose(3) = obj.goal(3);
                
                %Finds velocity components for specific direction travel
                %(the 0.5 is magnitude (not used to control vel))
                obj.vel = bodyToWorld([0.5;0;0],obj.pose);
                obj.state = 3;
            end
        end  
        
        function obj = moving(obj)% Moving the robot
            if distEu(obj.pose(1:2)',obj.goal(1:2)) > 0.05
                
                %This statement applies velocity
                obj.pose(1:2) = obj.pose(1:2) + obj.vel(1:2)* ... 
                    0.02; %This multiplier value controls velocity
            else
                obj.state = 4;
            end 
        end
        
        function obj = dispRob(obj,inA) %State display (debug)
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