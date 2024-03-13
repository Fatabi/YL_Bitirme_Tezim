classdef Map<handle
    properties
        startNode Node
        endNode Node
        map binaryOccupancyMap
        sx
        sy
    end
    methods
        function obj = Map(startNode,endNode,map)
            obj.startNode   = startNode;
            obj.endNode     = endNode;
            obj.map         = map;
            [obj.sy,obj.sx] = size(map.occupancyMatrix);
        end
    end
end