classdef robot < handle
    properties
        id = [];
        pose = [];
        state = [];
        
        goal = [];
        vel = [];
        %detected objects
        detObjs = [];
        %detected robots
        detRobs = [];
    end
    methods
        function obj = robot(N, M)
            obj.pose = N;
            obj.id = M;
        end
        function obj = fixPose(obj)
            obj.pose(3) = obj.pose(3) - (2 * pi * (obj.pose(3) / abs(obj.pose(3))));
        end
    end
    
end