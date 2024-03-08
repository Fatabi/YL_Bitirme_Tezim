classdef A_Star<handle

    properties
        map Map
        openSet Set % can be List of Nodes
    end

    methods
        function obj = A_Star(map)
            obj.map = map;
            obj.openSet = Set(obj.map.startNode);
        end
        function Solve(obj)
            while ~isempty(obj.openSet)
                currentNode = obj.openSet.findLowest_f_Score_Node();
                if currentNode.location == obj.map.endNode.location
                    %%% reconstruct_path(cameFrom, current)

                end
                obj.openSet.Remove(currentNode);
                currentNode.updateNeighbours(obj.map);
                for i = 1:length(currentNode.neighbours)
                    selectedNeighbour = currentNode.neighbours{i};
                    if ~isempty(selectedNeighbour)
                        if ~isempty(selectedNeighbour.location)
                            currentNode.calculate_g_Score();
                            tentative_g_Score = currentNode.gScore + Node.FindDistance(currentNode,selectedNeighbour);
                            selectedNeighbour.cameFrom = currentNode;
                            selectedNeighbour.calculate_g_Score();
                            if tentative_g_Score < selectedNeighbour.gScore
                                selectedNeighbour.cameFrom = currentNode;
                                selectedNeighbour.gScore = tentative_gScore;
                                selectedNeighbour.calculate_h_Score();
                                selectedNeighbour.calculate_f_Score();
                                if any(obj.openSet ~= selectedNeighbour)
                                    obj.openSet.Add(selectedNeighbour);
                                end
                            end
                        end

                    end
                end
            end
        end
    end
end

