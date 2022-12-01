% 定数入力を反映した時空間
classdef constantInputMetric < symbolicMetricSpace
    properties (Access = protected)
    end

    methods (Access = public)
        function obj = constantInputMetric(dim)    % コンストラクタ
            arguments
                dim % 時空間の次元．
            end
            obj@symbolicMetricSpace(dim);   % 親クラスのコンストラクタ呼び出し
        end

        function obj = setMetric(obj,A,B,C,f,c)
            % 計量の作成
            arguments
                obj
                A
                B
                C
                f
                c
            end
            obj.G(1,1) = f^2/c^4*A*(obj.x(1))^2-2*f/c^2*obj.x(1)*B+C;
            obj.G(1,2) = -f/c^2*A*obj.x(1)+B;
            obj.G(2,1) = obj.G(1,2);
            obj.G(2,2) = A;
            obj = obj.calcPartialG();   % 偏微分だけシンボリックで事前計算
        end
    end

    methods (Access = protected)
    end
end % classdef