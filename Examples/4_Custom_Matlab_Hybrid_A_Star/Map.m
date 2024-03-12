classdef Map<handle
    properties
        startNode Node
        endNode Node
        map binaryOccupancyMap
    end
    methods
        function obj = Map(startPoint,endPoint,map)
            obj.startNode = Node(startPoint);
            obj.endNode   = Node(endPoint);
            obj.map       = map;
            % occupancyMap  = obj.map.getOccupancy;
            % [sx,sy]       = size(occupancyMap);
            % obj.sx = sx;
            % obj.sy = sy;
            % obj.nodes = cell(sx,sy);
            % for i = 1:sx
            %     for j = 1:sy
            %         location = obj.idx_2_location([i,j]);
            %         if occupancyMap(i,j) == 1
            %             createdNode = Node([]);
            %         elseif all(location == startPoint)
            %             createdNode = obj.startNode;
            %         elseif all(location == endPoint)
            %             createdNode = obj.endNode;
            %         else
            %             createdNode = Node(location);
            %         end
            %         obj.nodes{i,j} = createdNode;
            %     end
            % end
        end
    end
end