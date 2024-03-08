classdef Node<handle
    properties
        location (1,:)
        neighbours
        cameFrom Node
        gScore = 0 % Cost from Start Node
        hScore = inf % Cost from End Node
        fScore = inf % Final Cost hScore + gScore
    end

    methods
        function obj = Node(location)
                obj.location = location;    
                obj.neighbours = cell(1,4);
        end
        function calculate_g_Score(obj)
            if (~isempty(obj.cameFrom))
                obj.gScore = obj.cameFrom.gScore + Node.FindDistance(obj,obj.cameFrom);
            end
        end
        function calculate_h_Score(obj,endNode)
            obj.hScore = Node.FindDistance(obj,endNode);
        end
        function calculate_f_Score(obj)
            obj.fScore = obj.gScore + obj.hScore;
        end
        function updateNeighbours(obj,map)
            i = obj.location(1);
            j = obj.location(2);

            obj.neighbours{1,1} = map.nodes{i-1,j};
            obj.neighbours{1,2} = map.nodes{i,j+1};
            obj.neighbours{1,3} = map.nodes{i+1,j};
            obj.neighbours{1,4} = map.nodes{i,j-1};
        end
    end
    methods(Static)
        function length = FindDistance(Node1,Node2)
                deltaLoc = (Node2.location - Node1.location);
                length = sqrt(deltaLoc*deltaLoc');
        end
    end
end

