classdef Map<handle
    properties
        startNode Node
        endNode Node
        map binaryOccupancyMap
    end
    methods
        function obj = Map(startNode,endNode,map)
            obj.startNode = startNode;
            obj.endNode   = endNode;
            obj.map       = map;
        end
    end
end