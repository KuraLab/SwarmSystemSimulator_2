classdef SyncronizingCoupledOscillator < Simulator
    %SyncronizingCoupledOscillator 結合振動子が同期するだけのシミュレーション
    %   

    properties
        % システムの変数を記載
        t_vec   % 時刻ベクトル
        phi     % 各振動子の位相
        Adj     % 隣接行列
    end

    methods
        function obj = SyncronizingCoupledOscillator()
            % コンストラクタ（宣言時に呼ばれる）
            obj@Simulator();    % 親クラスのコンストラクタも呼び出す
            obj = obj.setDefaultParameters();       % パラメタのデフォルト値を設定
        end
        
        function obj = setDefaultParameters(obj)
            % パラメータとデフォルト値を設定
            %%%%%%%% シミュレータの基本情報 %%%%%%%
            obj.param.dt = 0.01;    % 刻み時間
            obj.param.Nt = 1000;    % 計算するカウント数
            obj.param.Na = 5;       % エージェント数
            %%%%%%%% システムパラメータ %%%%%%%%
            obj.param.kappa = 1;    % 結合強度
            %%%%%%%%%%%%%% 初期値 %%%%%%%%%%%%%
            obj.param.omega_0 = [1; 1; 1; 1; 1];     % 固有角速度
            obj.param.phi_0 = [0.5; -0.2; 0.3; 0.4; -0.9]; % 初期位相
        end

        function obj = initializeVariables(obj)
            % 各種変数を初期化．シミュレーションをやり直す度に必ず呼ぶこと
            % 状態変数の定義と初期値の代入を行うこと
            obj.t_vec = 0:obj.param.dt:obj.param.dt*(obj.param.Nt-1); % 時刻ベクトルの定義
            obj.phi = zeros(obj.param.Na, obj.param.Nt);   % 状態変数phiの定義
            obj.phi(:,1) = obj.param.phi_0;                % 初期値の代入
            obj.Adj = [       % 隣接行列．お隣さんのみ観測可
                [0 1 0 0 0];
                [1 0 1 0 0];
                [0 1 0 1 0];
                [0 0 1 0 1];
                [0 0 0 1 0];
                ];
        end

        function obj = simulate(obj)
            % シミュレーション本体
            disp("シミュレーションを開始します...")
            for t = 1:obj.param.Nt-1
                for n = 1:obj.param.Na
                    phi_ij = obj.phi(:,t).' - obj.phi(n,t); %位相差
                    % 差分方程式の更新．隣接エージェントとの位相差を足し合わせて結合強度をかける．台数で正規化していないので注意
                    obj.phi(n,t+1) = obj.phi(n,t) + obj.param.dt*( obj.param.omega_0(n,1)+ obj.param.kappa*sum( obj.Adj(n,:).*phi_ij, 2));  
                end
                % 上手く行列計算すればnのfor文はなくせる
            end % for
        end % simulate

        function obj = plot(obj)
            % 各種プロット
            disp("描画を開始します...")

            figure  % 生値プロット
            plot(obj.t_vec, obj.phi);
            title("位相の時間変化（生値）")
            xlabel("時刻 t [s]")
            ylabel("位相")
            legend(string(1:obj.param.Na))

            figure;  % 生値プロット
            plot(obj.t_vec, obj.phi-obj.phi(1,:));
            title("位相の時間変化（1番エージェント基準）")
            xlabel("時刻 t [s]")
            ylabel("位相")
            legend(string(1:obj.param.Na))
        end

        function obj = subplot(obj)
            % 一括描画用の最低限プロット
            plot(obj.t_vec, obj.phi-obj.phi(1,:));
        end

    end % methods
end % class