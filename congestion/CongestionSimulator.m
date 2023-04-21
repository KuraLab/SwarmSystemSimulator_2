
% 1次元の渋滞シミュレータ

classdef CongestionSimulator< Simulator
    properties
        % システムの変数を記載
        t_vec   % 固有時刻ベクトル
        x       % 状態変数
        dxdt    % 1階微分
    end

    methods
        function obj = CongestionSimulator()
            % コンストラクタ（宣言時に呼ばれる）
            obj@Simulator();    % 親クラスのコンストラクタも呼び出す
            obj = obj.setDefaultParameters();       % パラメタのデフォルト値を設定
        end

        function obj = setDefaultParameters(obj)
            % パラメータとデフォルト値を設定
            %%%%%%%% シミュレータの基本情報 %%%%%%%
            obj.param.dt = 0.05;    % 刻み時間
            obj.param.Nt = 400;    % 計算するカウント数
            obj.param.Na = 3;       % エージェント数
            %%%%%%%%%%%%% 環境情報 %%%%%%%%%%%%
            obj.param.L = 10;       % 周長
            %%%%%%%% システムパラメータ %%%%%%%%
            obj.param.control_model = "sugiyama";
            obj.param.k = 1;       % ゲイン
            obj.param.c_0 = 0;       % 初速度
            %%%%%%%%%%%%%% 初期値 %%%%%%%%%%%%%
            %obj.param.x_0 = 0:obj.param.L/obj.param.Na:obj.param.L/obj.param.Na*(obj.param.Na-1);   % 等配分．摂動はこれに足しこむ形が良い
            %obj.param.dxdt_0 = obj.param.c*ones(obj.param.Na,1);    % 初速度
        end
        
        function obj = initializeVariables(obj)
            % 各種変数を初期化．シミュレーションをやり直す度に必ず呼ぶこと
            % 状態変数の定義と初期値の代入を行うこと
            obj.t_vec = 0:obj.param.dt:obj.param.dt*(obj.param.Nt); % 時刻ベクトルの定義
            obj.x(:,:) = zeros(obj.param.Na, obj.param.Nt);    % 状態変数の定義
            obj.dxdt(:,:) = zeros(obj.param.Na, obj.param.Nt);    % 状態変数の定義
            obj.x(:,1) = 0:obj.param.L/obj.param.Na:obj.param.L/obj.param.Na*(obj.param.Na-1);   % 等配分．摂動はこれに足しこむ形が良い
            obj.dxdt(:,1) = obj.param.c_0*ones(obj.param.Na,1);    % 初速度
        end

        function obj = defineSystem(obj)
            % システムの定義が必要な場合はここ．シミュレーションをやり直すたびに呼ぶこと

        end

        function obj = simulate(obj)
            % シミュレーション本体
            disp("シミュレーションを開始します...")
            tic
            for t = 1:obj.param.Nt
                % ループ毎の更新をここに
                u_nominal = zeros(obj.param.Na,1);  % ノミナル入力
                if obj.param.control_model == "sugiyama"
                    u_nominal = obj.sugiyamaController(t);
                end
                u = u_nominal;
                obj.x(:,t+1) = obj.x(:,t) + obj.param.dt*obj.dxdt(:,t); % 1次オイラー差分
                obj.dxdt(:,t+1) = obj.dxdt(:,t) + obj.param.dt*u;
                obj.x(:,t+1) = obj.x(:,t+1) - (obj.x(:,t+1)>obj.param.L).*obj.param.L + (obj.x(:,t+1)<0).*obj.param.L;  % 周期条件．rem関数で良さそう
            end % for
            toc
        end % simulate
        
        %%%%%%%%%%%%%%%%%%%% 制御器 %%%%%%%%%%
        function u_nominal = sugiyamaController(obj, t)
            arguments
                obj
                t       % 時刻
            end
            u_nominal = zeros(obj.param.Na,1);
            Delta_x = [obj.x(2:end,t);obj.x(1,t)]-obj.x(:,t);   % 前方エージェントとの差を返すNa*1ベクトル
            Delta_x = Delta_x + (Delta_x<0).*obj.param.L;       % 前方エージェントとの差が負になった場合． 周期条件を適用して正に戻す
        end

        %%%%%%%%%%%%%%%%%%%%%%%% 描画 %%%%%%%%

        function obj = positionLinearPlot(obj)
            % 各種プロット
            disp("描画を開始します...")

            figure  % 生値プロット
            plot(obj.t_vec, obj.x(:,:));
            title("エージェント位置の時間変化")
            xlabel("時刻 t [s]")
            legend(string(1:obj.param.Na))
            ylim([0, obj.param.L])
        end

        function obj = subplot(obj)
            % 一括描画用の最低限プロット
            % plot(obj.t_vec, obj.x(:,:));
            % xlabel("時刻 t [s]")
            % legend(["1","2","3"])
        end

    end % methods
end % class