classdef spaceViewSimulator< Simulator
% 時空間のシミュレータ
    properties
        % システムの変数を記載
        t_vec   % 固有時刻ベクトル
        x       % 状態変数
        dxdt
    end

    methods
        function obj = spaceViewSimulator()
            % コンストラクタ（宣言時に呼ばれる）
            obj@Simulator();    % 親クラスのコンストラクタも呼び出す
            obj = obj.setDefaultParameters();       % パラメタのデフォルト値を設定
        end

        function obj = setDefaultParameters(obj)
            % パラメータとデフォルト値を設定
            %%%%%%%% シミュレータの基本情報 %%%%%%%
            obj.param.dt = 0.05;    % 刻み時間
            obj.param.Nt = 400;    % 計算するカウント数
            %%%%%%%% システムパラメータ %%%%%%%%
            obj.param.k = 0.1;       % ゲイン
            %%%%%%%%%%%%%% 初期値 %%%%%%%%%%%%%
            obj.param.x_0 = [0; 0];
            obj.param.dxdt_0 = [0.5; 1];
        end
        
        function obj = initializeVariables(obj)
            % 各種変数を初期化．シミュレーションをやり直す度に必ず呼ぶこと
            % 状態変数の定義と初期値の代入を行うこと
            obj.t_vec = 0:obj.param.dt:obj.param.dt*(obj.param.Nt-1); % 時刻ベクトルの定義
            obj.x(:,:) = zeros(2, obj.param.Nt-1);    % 状態変数の定義
            obj.dxdt(:,:) = zeros(2, obj.param.Nt-1);    % 状態変数の定義
            obj.x(:,1) = obj.param.x_0;   % 初期値の代入
            obj.dxdt(:,1) = obj.param.dxdt_0;   % 初期値の代入
        end

        function obj = defineSystem(obj)
            % システムの定義が必要な場合はここ．シミュレーションをやり直すたびに呼ぶこと

        end

        function obj = simulate(obj)
            % シミュレーション本体
            disp("シミュレーションを開始します...")
            tic
            for t = 1:obj.param.Nt-1
                % ループ毎の更新をここに
                % u = obj.param.K*obj.x(:,t);
                obj.x(:,t+1) = obj.x(:,t) + obj.param.dt*obj.dxdt(:,t);
                obj.dxdt(:,t+1) = obj.dxdt(:,t) + obj.param.k*obj.param.dt*[0; -1];
            end % for
            toc
        end % simulate

        function obj = plot(obj)
            % 各種プロット
            disp("描画を開始します...")

            figure  % 生値プロット
            plot(obj.x(1,:), obj.x(2,:));
            title("状態の時間変化")
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
            plot(obj.x(1,:), obj.x(2,:),'Color',"#0072BD");
            hold on
            plot(obj.x(1,t), obj.x(2,t), 'o', 'Color','r','MarkerSize',10,'MarkerFaceColor','r');
            text(9,5,"t = "+sprintf("%02.1f",t*obj.param.dt),'FontSize',20);
            xlim([0,12]);
            ylim([0,6]);
            xlabel("x");
            ylabel("y");
            hold off
        end

        function moviePlot2(obj,t)
            % 動画用のフレーム単位プロット
            plot3(obj.x(1,:), 20*ones(obj.param.Nt,1), obj.x(2,:),'Color',	"#D95319");
            hold on
            plot3(obj.x(1,:),obj.t_vec, obj.x(2,:),'Color',"#0072BD");
            plot3(obj.x(1,t),obj.t_vec(t),  obj.x(2,t), 'o', 'Color','r','MarkerSize',10,'MarkerFaceColor','r');
            ylim([0,20]);
            xlim([0,12]);
            zlim([0,6]);
            ylabel("t");
            xlabel("x");
            zlabel("y");
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