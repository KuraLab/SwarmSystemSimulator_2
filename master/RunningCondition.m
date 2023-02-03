classdef RunningCondition
    %RunningCondition 繰り返し条件の指定
    %   パラメータを変えながら複数回の試行を行うようなシミュレーションにおいて，走らせるパラメータ情報を格納するクラス

    properties
        condition_name (1,1) {mustBeText} = ""   % この条件の名称．結果tableの列名に該当
        parameter_name (1,1) {mustBeText} = ""   % 操作するパラメータ名
        vals = []                                % パラメータに代入する値の配列．結果tableに記録される
        vals_name   {mustBeText} = ""            % 各パラメータが意味すること．""ならvalsがそのまま
        condition_length = 0                              % 繰り返し長さ
    end

    methods
        function obj = RunningCondition()
            %RunningCondition Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function obj = setParameter(obj, cond_name_, param_name_)
            % 操作対象のパラメータを指定
            arguments
                obj
                cond_name_      % 条件の名称
                param_name_     % 操作するパラメータ名
            end
            obj.condition_name = cond_name_;
            obj.parameter_name = param_name_;
        end

        function obj = setRunningValues(obj,vals_)
            % パラメータに代入する値を決定
            arguments
                obj
                vals_      % パラメータに代入する値の配列
            end
            obj.vals = vals_;
            obj.vals_name = string(vals_);  % 一旦各条件名には代入値を入れておく
            obj.condition_length = length(vals_);
        end

        function obj = setRunningNames(obj,names_)
            % 各パラメータ条件の名称を指定
            arguments
                obj
                names_      % パラメータに代入する値の配列
            end
            obj.vals_name = names_;
        end
    end
end