classdef robot < handle
    
    %% Public Properties
    properties
        id = [];
        pose = [];
    end
    
    %% Public Methods
    methods
        %Initaliser
        function obj = robot(N, M)
            obj.pose = N;
            obj.id = M;
        end
    end
end