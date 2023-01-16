% ドジッター空間に入力
classdef deSitterMetric < symbolicMetricSpace
    properties (Access = protected)
    end

    methods (Access = public)
        function obj = deSitterMetric(dim)    % コンストラクタ
            arguments
                dim % 時空間の次元．
            end
            obj@symbolicMetricSpace(dim);   % 親クラスのコンストラクタ呼び出し
        end

        function obj = setMetric(obj,f,c)
            % 計量の作成
            arguments
                obj
                f
                c
            end
            obj.G(1,1) = 1;
            obj.G(1,2) = 0;
            obj.G(2,1) = obj.G(1,2);
            obj.G(2,2) = -(cosh(obj.x(1)))^2;
            obj = obj.calcPartialG();   % 偏微分だけシンボリックで事前計算
        end
    end

    methods (Access = protected)
    end
end % classdef