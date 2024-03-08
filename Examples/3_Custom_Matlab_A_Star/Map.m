classdef Map<handle
    properties
        startNode Node
        endNode Node
        map binaryOccupancyMap
        nodes
    end
    methods
        function obj = Map(startPoint,endPoint,map)
           obj.startNode = Node(startPoint);
           obj.endNode   = Node(endPoint);
           obj.map       = map;
           occupancyMap  = obj.map.getOccupancy;
           [sx,sy]       = size(occupancyMap);
           obj.nodes = cell(sx,sy);
           for i = 1:sx
               for j = 1:sy
                   location = [i,j];
                    if occupancyMap(i,j) == 1
                        createdNode = Node([]);
                    else
                        createdNode = Node(location);
                    end
               obj.nodes{i,j} = createdNode;
               end
           end
        end
    end
end

