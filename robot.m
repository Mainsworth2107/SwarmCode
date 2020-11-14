%% Class that stores the poses of each robot
 
classdef robot < handle
    
    %% Public Properties
    properties
        id = [];
        pose = [];
    end
    
    %% Public Methods
    methods
        %Initialiser
        function obj = robot(N, M)
            obj.pose = N;
            obj.id = M;
        end
    end
end