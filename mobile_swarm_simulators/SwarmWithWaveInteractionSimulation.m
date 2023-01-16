
classdef SwarmWithWaveInteractionSimulation < MobileRobots2dSimulator
    
    properties
        cbf
    end

    methods

        function obj = SwarmWithWaveInteractionSimulation()
            % コンストラクタ（宣言時に呼ばれる）
            obj@MobileRobots2dSimulator();    % 親クラスのコンストラクタも呼び出す
            obj = obj.setDefaultParameters();       % パラメタのデフォルト値を設定
            obj.cbf = CollisionAvoidanceCBF();
        end

        function obj = setDefaultParameters(obj)
            obj = obj.setDefaultParameters@MobileRobots2dSimulator();   % スーパークラス側の読み出し
            obj.param.kp = 1;   % 勾配追従力
        end
        
        function obj = initializeVariables(obj)
            % 各種変数を初期化．シミュレーションをやり直す度に必ず呼ぶこと
            % 状態変数の定義と初期値の代入を行うこと
            obj = obj.initializeVariables@MobileRobots2dSimulator();   % スーパークラス側の読み出し
        end

        function obj = calcControlInput(obj,t)
            % 入力の生成．継承して使う
            arguments
                obj
                t    % 時刻
            end
            
            u_t = zeros(obj.param.Na, 2);   % 時刻tにおける入力

            u_nominal(1,:) = obj.param.kp*(obj.x(2,:,t)-obj.x(1,:,t));
            u_nominal(2,:) = obj.param.kp*(obj.x(1,:,t)-obj.x(2,:,t));
            obj.cbf = obj.cbf.setParameters(1,1,obj.param.dt,1,true);
            % 1番用CBF
            obj.cbf = obj.cbf.addConstraints(obj.x(2,:,t)-obj.x(1,:,t), obj.dxdt(2,:,t)-obj.dxdt(1,:,t));
            u_t(1,:) = obj.cbf.apply(u_nominal(1,:));
            obj.cbf = obj.cbf.clearConstraints();
            % 2番用CBF
            obj.cbf = obj.cbf.addConstraints(obj.x(1,:,t)-obj.x(2,:,t), obj.dxdt(1,:,t)-obj.dxdt(2,:,t));
            u_t(2,:) = obj.cbf.apply(u_nominal(2,:));
            obj.cbf = obj.cbf.clearConstraints();
            
            %u_t(12,:) = [0.1,0.1];
            obj.u(:,:,t) = u_t;
        end
    end

end % clasdef