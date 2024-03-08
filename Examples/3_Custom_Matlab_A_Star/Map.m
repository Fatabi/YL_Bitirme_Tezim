classdef Map<handle
    properties
        startNode Node
        endNode Node
        map binaryOccupancyMap
        nodes
        sx
        sy
    end
    methods
        function obj = Map(startPoint,endPoint,map)
            obj.startNode = Node(startPoint);
            obj.endNode   = Node(endPoint);
            obj.map       = map;
            occupancyMap  = obj.map.getOccupancy;
            [sx,sy]       = size(occupancyMap);
            obj.sx = sx;
            obj.sy = sy;
            obj.nodes = cell(sx,sy);
            for i = 1:sx
                for j = 1:sy
                    location = obj.idx_2_location([i,j]);
                    if occupancyMap(i,j) == 1
                        createdNode = Node([]);
                    elseif all(location == startPoint)
                        createdNode = obj.startNode;
                    elseif all(location == endPoint)
                        createdNode = obj.endNode;
                    else
                        createdNode = Node(location);
                    end
                    obj.nodes{i,j} = createdNode;
                end
            end
        end
        function idx = location_2_idx(obj,loc)
            %loc 0  ,0      = 100,1   idx
            %loc 99 ,0      = 100,100 idx
            %loc 0  ,99     = 1  ,1   idx
            %loc 99 ,99     = 1  ,100 idx
            idx = [(obj.sx - loc(2)) , (loc(1)+1)];
        end
        function loc = idx_2_location(obj,idx)
            loc = [idx(2)-1,-(idx(1)-obj.sx)];
        end
    end
end