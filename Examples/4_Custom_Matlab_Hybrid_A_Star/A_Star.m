classdef A_Star<handle

    properties
        map Map
        openSet Set % can be List of Nodes
        closedSet Set % can be List of Nodes
        pathLoc = [];
        drawNow = false
        pathLine = [];
        branchLine = [];
        closeEnoguhDistance = 2
    end

    methods
        function obj = A_Star(map,opt)
            arguments
                map
                opt.drawNow = false
                opt.pathLine = []
                opt.branchLine = []
            end
            obj.map         = map;
            obj.openSet     = Set(obj.map.startNode);
            obj.closedSet   = Set(Node.empty);
            obj.drawNow     = opt.drawNow;
            obj.pathLine    = opt.pathLine;
            obj.branchLine  = opt.branchLine;
        end
        function Solve(obj)
            while ~isempty(obj.openSet)
                currentNode = obj.openSet.findLowest_f_Score_Node();
                dist = Node.FindDistance(currentNode.state(1:2),obj.map.endNode.state(1:2));
                if dist < obj.closeEnoguhDistance
                    disp("Path is found.")
                    obj.openSet.Remove(currentNode);
                    break
                end
                obj.openSet.Remove(currentNode);
                obj.closedSet.Add(currentNode);
                currentNode.createNeighbours(obj.map);
                for i = 1:length(currentNode.neighbours)
                    selectedNeighbour = currentNode.neighbours{i};
                    if ~isempty(selectedNeighbour)
                            if ~any(obj.closedSet.set == selectedNeighbour)
                                selectedNeighbour.cameFrom = currentNode;
                                currentNode.calculate_g_Score();
                                tentative_g_Score = currentNode.gScore + Node.FindDistance(currentNode.state(1:2),selectedNeighbour.state(1:2));

                                if  ~any(obj.openSet.set == selectedNeighbour)
                                    obj.openSet.Add(selectedNeighbour)
                                end
                                
                                selectedNeighbour.calculate_f_Score(obj.map.endNode.state(1:2));
                                selectedNeighbour.cameFrom = currentNode;
                                
                            else
                                % disp("In closed set")
                            end
                    else
                        % disp("a")
                    end
                end
            
                if obj.drawNow && ~isempty(obj.pathLine)
                    tempXdata = obj.pathLine.XData;
                    tempYdata = obj.pathLine.YData;
                    obj.pathLine.XData = [tempXdata,currentNode.state(1)];
                    obj.pathLine.YData = [tempYdata,currentNode.state(2)];
                    drawnow();
                end
                if obj.drawNow && ~isempty(obj.branchLine)                                     
                    for ii = 1:length(currentNode.neighbours)
                        tempXdata = obj.branchLine.XData;
                        tempYdata = obj.branchLine.YData;
                        if (~isempty(currentNode.statesOfBranches{ii}))
                        obj.branchLine.XData = [tempXdata,currentNode.statesOfBranches{ii}(:,1)'];
                        obj.branchLine.YData = [tempYdata,currentNode.statesOfBranches{ii}(:,2)'];
                        end
                    end
                    drawnow();   
                end
            end
        end
        function pathLoc = CreatePathLocation(obj)
            selectedNode = obj.openSet.set(end).cameFrom;
            pathLoc = [];
            while ~isempty(selectedNode)
                pathLoc = [pathLoc ;selectedNode.state(1:2)];
                selectedNode = selectedNode.cameFrom;
            end
            obj.pathLoc = pathLoc;
        end
    end
end

