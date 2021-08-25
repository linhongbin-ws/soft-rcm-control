classdef LPF < handle
    %Low pass filter
    
    properties
        prv_value = [];
        filter_ratio %0~1
        new_val = [];
    end
    
    methods
        function obj = LPF(filter_ratio)
            obj.filter_ratio = filter_ratio;
            obj.reset();
        end
        
        function new_val = update(obj, input_value)
            if isempty(obj.prv_value)
                obj.prv_value = input_value;
                new_val = input_value;
            else
                new_val = input_value * (1-obj.filter_ratio) + obj.prv_value * obj.filter_ratio;
                obj.prv_value = obj.new_val;
            end
            obj.new_val = new_val;   
        end
        
        function reset(obj)
            obj.prv_value = [];
            obj.new_val = [];
        end
    end
end

