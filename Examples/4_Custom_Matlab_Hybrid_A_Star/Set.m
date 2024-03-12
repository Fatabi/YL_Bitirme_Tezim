classdef Set<handle
    properties
        set Node
    end
    
    methods
        function obj = Set(nodes)
            obj.set = nodes;
        end        
        function lowest_f_Score_Node = findLowest_f_Score_Node(obj)
            lowest_f_score = inf;           % Place Holder
            lowest_f_Score_Node = obj.set(1);   % Place Holder

            for i = 1:length(obj.set)
                if obj.set(i).fScore < lowest_f_score
                   lowest_f_Score_Node = obj.set(i);
                   lowest_f_score = obj.set(i).fScore;
                end
            end
        end
        function Remove(obj,node)
            obj.set(obj.set == node) = [];
        end
        function Add(obj,node)
            obj.set = [obj.set,node];
        end
    end
end

