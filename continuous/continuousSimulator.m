classdef continuousSimulator< Simulator
% 時空間のシミュレータ
    properties
        % システムの変数を記載
        t_vec   % 固有時刻ベクトル
        x_vec
        rho       % 状態変数
        v
        phi
        Nabla   % 勾配を計算するための線形作用素
        Lap   % ラプラシアンを計算するための線形作用素
    end

    methods
        function obj = continuousSimulator()
            % コンストラクタ（宣言時に呼ばれる）
            obj@Simulator();    % 親クラスのコンストラクタも呼び出す
            obj = obj.setDefaultParameters();       % パラメタのデフォルト値を設定
        end

        function obj = setDefaultParameters(obj)
            % パラメータとデフォルト値を設定
            %%%%%%%% シミュレータの基本情報 %%%%%%%
            obj.param.dt = 0.05;    % 刻み時間
            obj.param.dx = 0.1;    % 刻み距離
            obj.param.Nt = 300;    % 計算するカウント数
            obj.param.Nx = 100;     % 空間方向の点数
            %%%%%%%% システムパラメータ %%%%%%%%
            obj.param.k = 0.01;       % 勾配追従力ゲイン
            obj.param.kappa = 2;      % 結合強度
            %%%%%%%%%%%%%% 初期値 %%%%%%%%%%%%%
            obj.param.rho_0 = ones(obj.param.Nx,1);
            obj.param.v_0 = zeros(obj.param.Nx,1);
            obj.param.phi_0 = zeros(obj.param.Nx,1);
        end
        
        function obj = initializeVariables(obj)
            % 各種変数を初期化．シミュレーションをやり直す度に必ず呼ぶこと
            % 状態変数の定義と初期値の代入を行うこと
            obj.t_vec = 0:obj.param.dt:obj.param.dt*(obj.param.Nt-1); % 時刻ベクトルの定義
            obj.x_vec = 0:obj.param.dx:obj.param.dx*(obj.param.Nx-1); % 位置ベクトルの定義
            obj.rho(:,:) = zeros(obj.param.Nx, obj.param.Nt);    % 状態変数の定義
            obj.v(:,:) = zeros(obj.param.Nx, obj.param.Nt);    % 状態変数の定義
            obj.phi(:,:) = zeros(obj.param.Nx, obj.param.Nt);
            obj.rho(:,1) = obj.param.rho_0;   % 初期値の代入
            obj.v(:,1) = obj.param.v_0;   % 初期値の代入
            obj.phi(:,1) = obj.param.phi_0;
        end

        function obj = defineSystem(obj)
            % システムの定義が必要な場合はここ．シミュレーションをやり直すたびに呼ぶこと
            obj.Nabla = zeros(obj.param.Nx);    % Nx*Nx正方行列
            up_flow = (diag(ones(obj.param.Nx-1,1),1)+diag(-1*ones(obj.param.Nx,1),0))/obj.param.dx;    % 風上差分:対角成分-1, 
            down_flow = -1*up_flow.';   % 風下差分
            down_flow(1,1) = 0;    % 端っこの補正
            up_flow(obj.param.Nx,obj.param.Nx) = 0;
            obj.Nabla = (up_flow+down_flow)/2;    % 勾配は，風上差分と風下差分の平均
            obj.Lap = (up_flow-down_flow)/obj.param.dx;    % 発散は，風上差分と風下差分の差
        end

        function obj = simulate(obj)
            % シミュレーション本体
            disp("シミュレーションを開始します...")
            tic
            for t = 1:obj.param.Nt-1
                % ループ毎の更新をここに
                % u = obj.param.K*obj.x(:,t);
                obj.rho(:,t+1) = obj.rho(:,t) - obj.param.dt*obj.Nabla*(obj.rho(:,t).*obj.v(:,t));
                obj.v(:,t+1) = obj.v(:,t) + obj.param.k*obj.param.dt*obj.Nabla*obj.phi(:,t);
                obj.phi(:,t+1) = obj.phi(:,t) + obj.param.kappa*obj.param.dt*obj.Lap*obj.phi(:,t);
            end % for
            toc
        end % simulate

        function obj = plot(obj)
            % 各種プロット
            disp("描画を開始します...")

            figure  % 生値プロット
            plot(obj.t_vec, obj.rho(:,:));
            title("状態の時間変化")
            legend(string(1:obj.param.Nx));
            xlabel("時刻 t [s]")
        end

        function obj = subplot(obj)
            % 一括描画用の最低限プロット
            % plot(obj.t_vec, obj.x(:,:));
            % xlabel("時刻 t [s]")
            % legend(["1","2","3"])
        end

        function moviePlot(obj,t)
            % 動画用のフレーム単位プロット
            subplot(1,2,1)
            plot(obj.x_vec, obj.rho(:,t).','Color',"#0072BD");
            %xlim([0,12]);
            ylim([min(obj.rho,[],'all'),max(obj.rho,[],'all')]);
            ylabel("rho");
            xlabel("x");
            hold off

            subplot(1,2,2)
            plot(obj.x_vec, obj.phi(:,t).','Color',"#0072BD");
            hold on
            %xlim([0,12]);
            ylim([0,1]);
            ylabel("phi");
            xlabel("x");

            text(obj.x_vec(end)*0.7, 0.8, "t = "+string(t), 'FontSize',12);
            hold off
        end

        function moviePlot2(obj,t)
            % 動画用のフレーム単位プロット
            plot(obj.x_vec, obj.phi(:,t).','Color',"#0072BD");
            %xlim([0,12]);
            ylim([0,1]);
            ylabel("phi");
            xlabel("x");
            hold off
        end
        
        function obj = generateMovie(obj, filename, speed)
            arguments
                obj
                filename string = "movie.mp4" % 保存するファイル名
                speed = 1       % 動画の再生速度
            end
            obj.makeMovie(@obj.moviePlot, obj.param.dt, obj.param.Nt, filename, speed);
        end

        function obj = generateMovie2(obj, filename, speed)
            arguments
                obj
                filename string = "movie2.mp4" % 保存するファイル名
                speed = 1       % 動画の再生速度
            end
            obj.makeMovie(@obj.moviePlot2, obj.param.dt, obj.param.Nt, filename, speed);
        end

    end % methods
end % class