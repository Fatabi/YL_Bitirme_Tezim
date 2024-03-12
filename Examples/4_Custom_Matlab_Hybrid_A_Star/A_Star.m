classdef A_Star<handle

    properties
        map Map
        openSet Set % can be List of Nodes
        closedSet Set % can be List of Nodes
        pathLoc = [];
        drawNow = false
        pathLine = [];
    end

    methods
        function obj = A_Star(map,opt)
            arguments
                map
                opt.drawNow = false
                opt.pathLine = []
            end
            obj.map         = map;
            obj.openSet     = Set(obj.map.startNode);
            obj.closedSet   = Set(Node.empty);
            obj.drawNow     = opt.drawNow;
            obj.pathLine    = opt.pathLine;
        end
        function Solve(obj)
            while ~isempty(obj.openSet)
                currentNode = obj.openSet.findLowest_f_Score_Node();
                % disp(currentNode.location)
                % pause(0.2)
                if currentNode.location == obj.map.endNode.location
                    disp("Path is found.")
                    obj.openSet.Remove(currentNode);
                    break
                end
                obj.openSet.Remove(currentNode);
                obj.closedSet.Add(currentNode);
                currentNode.updateNeighbours(obj.map);
                for i = 1:length(currentNode.neighbours)
                    selectedNeighbour = currentNode.neighbours{i};
                    if ~isempty(selectedNeighbour)
                        if ~isempty(selectedNeighbour.location)
                            if ~any(obj.closedSet.set == selectedNeighbour)
                                selectedNeighbour.cameFrom = currentNode;
                                currentNode.calculate_g_Score();
                                tentative_g_Score = currentNode.gScore + Node.FindDistance(currentNode,selectedNeighbour);

                                if  ~any(obj.openSet.set == selectedNeighbour)
                                    obj.openSet.Add(selectedNeighbour)
                                end
                                % elseif tentative_g_Score < selectedNeighbour.gScore
                                selectedNeighbour.gScore = tentative_g_Score;
                                selectedNeighbour.calculate_h_Score(obj.map.endNode);
                                selectedNeighbour.calculate_f_Score();
                                selectedNeighbour.cameFrom = currentNode;
                                % end
                            else
                                % disp("In closed set")
                            end
                        end
                    end
                end
            
                if obj.drawNow && ~isempty(obj.pathLine)
                    tempXdata = obj.pathLine.XData;
                    tempYdata = obj.pathLine.YData;
                    obj.pathLine.XData = [tempXdata,currentNode.location(1)];
                    obj.pathLine.YData = [tempYdata,currentNode.location(2)];
                    drawnow();
                end
            end
        end
        function pathLoc = CreatePathLocation(obj)
            selectedNode = obj.openSet.set(end).cameFrom;
            pathLoc = [];
            while ~isempty(selectedNode)
                pathLoc = [pathLoc ;selectedNode.location];
                selectedNode = selectedNode.cameFrom;
            end
            obj.pathLoc = pathLoc;
        end
    end
end

