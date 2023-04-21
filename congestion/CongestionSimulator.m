
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
            obj.param.Nt = 800;    % 計算するカウント数
            obj.param.Na = 10;       % エージェント数
            %%%%%%%%%%%%% 環境情報 %%%%%%%%%%%%
            obj.param.L = 10;       % 周長
            %%%%%%%% システムパラメータ %%%%%%%%
            %obj.param.control_model = "sugiyama";   % 制御モデルの指定
            obj.param.control_model = "velocity_FB";   % 制御モデルの指定
            % 杉山モデル
            obj.param.k = 0;       % ゲイン
            obj.param.c_0 = 0;       % 初速度
            obj.param.r_s = obj.param.L/obj.param.Na*0.3;   % 等間隔の30%
            obj.param.a = 0;        % 時定数
            % 線形速度FBモデル
            obj.param.v_r = 0;      % 目標速度
            obj.param.k_v = 0;      % 追従ゲイン
            % 車間PD制御モデル
            obj.param.k_P = 0;      % 車間(P)ゲイン
            obj.param.k_D = 0;      % 速度差(D)ゲイン
            obj.param.r_r = 0;      % 目標車間
            % CBFパラメータ
            obj.param.CBF_enable = false;   % CBF使う？
            obj.param.Tc = 0;       % 時定数ゲイン(h0->h1)
            obj.param.gamma = 0;    % 粘性ゲイン（h1->h2）
            % 外乱
            obj.param.epsilon = 0;    % 初期位置ずらし量
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
            obj.x(1,1) = obj.x(1,1) + obj.param.epsilon * obj.param.L/obj.param.Na;   % 初期間隔のepsilon倍，あるエージェントをずらす
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
                elseif obj.param.control_model == "velocity_FB"
                    u_nominal = obj.velocityFBController(t);
                elseif obj.param.control_model == "PD"
                    u_nominal = obj.PDController(t);
                end
                u = u_nominal;
                obj.x(:,t+1) = obj.x(:,t) + obj.param.dt*obj.dxdt(:,t); % 1次オイラー差分
                obj.dxdt(:,t+1) = obj.dxdt(:,t) + obj.param.dt*u;
                obj.x(:,t+1) = obj.x(:,t+1) - (obj.x(:,t+1)>obj.param.L).*obj.param.L + (obj.x(:,t+1)<0).*obj.param.L;  % 周期条件．rem関数で良さそう
            end % for
            toc
        end % simulate
        
        %%%%%%%%%%%%%%%%%%%% 制御器 %%%%%%%%%%
        function u_nominal = sugiyamaController(obj, t) % M. Bando et. al. PHYSICS REVIEW E (1995)を参照 
            arguments
                obj
                t       % 時刻
            end
            u_nominal = zeros(obj.param.Na,1);
            Delta_x = [obj.x(2:end,t);obj.x(1,t)]-obj.x(:,t);   % 前方エージェントとの差を返すNa*1ベクトル
            Delta_x = Delta_x + (Delta_x<0).*obj.param.L;       % 前方エージェントとの差が負になった場合． 周期条件を適用して正に戻す
            V = obj.param.k*(Delta_x-obj.param.r_s);
            u_nominal = obj.param.a*(V-obj.dxdt(:,t));
        end

        function u_nominal = velocityFBController(obj, t)   % 速度FB，これだけだとぶつかる
            arguments
                obj
                t       % 時刻
            end
            u_nominal = obj.param.k_v*(obj.param.v_r-obj.dxdt(:,t));    % 速度をvrに保つ
        end

        function u_nominal = PDController(obj, t)   % 車間PD制御，kpを0にしたら車間を保つ制御
            arguments
                obj
                t       % 時刻
            end
            Delta_x = [obj.x(2:end,t);obj.x(1,t)]-obj.x(:,t);   % 前方エージェントとの差を返すNa*1ベクトル
            Delta_x = Delta_x + (Delta_x<0).*obj.param.L;       % 前方エージェントとの差が負になった場合． 周期条件を適用して正に戻す
            u_nominal = obj.param.k_P*(Delta_x-obj.param.r_r) + obj.param.k_D*([obj.dxdt(2:end,t);obj.dxdt(1,t)]-obj.dxdt(:,t));    % 速度をvrに保つ
        end

        %%%%%%%%%%%%%%%%%%%%%%%% 描画 %%%%%%%%

        function obj = positionLinearPlot(obj)
            % 基本プロット
            disp("描画を開始します...")

            figure  % 生値プロット
            plot(obj.t_vec, obj.x(:,:));
            title("エージェント位置の時間変化")
            xlabel("時刻 t [s]")
            legend(string(1:obj.param.Na))
            ylim([0, obj.param.L])
            title(obj.titleSet(),'Interpreter','latex')
        end

        function obj = headwayPlot(obj)
            % 車間プロット
            figure
            plot(obj.t_vec, mod([obj.x(2:end,:);obj.x(1,:)]-obj.x(:,:),obj.param.L));
            title("エージェント間隔の時間変化")
            xlabel("時刻 t [s]")
            legend(string(1:obj.param.Na))
            ylim([0, obj.param.L])
            title(obj.titleSet(),'Interpreter','latex')
        end

        function title_ = titleSet(obj) % 図のタイトルにパラメータを入れる場合
            if obj.param.control_model == "sugiyama"
                title_ = "$f/a = "+string(obj.param.k/obj.param.a)+"$";
            elseif obj.param.control_model == "velocity_FB"
                title_ = "$v_r = "+string(obj.param.v_r)+",k_v = "+string(obj.param.k_v)+"$";
            elseif obj.param.control_model == "PD"
                title_ = "$k_P = "+string(obj.param.k_P)+",k_D = "+string(obj.param.k_D)+",r_r = "+string(obj.param.r_r)+"$";
            end
        end

        function obj = subplot(obj)
            % 一括描画用の最低限プロット
            % plot(obj.t_vec, obj.x(:,:));
            % xlabel("時刻 t [s]")
            % legend(["1","2","3"])
        end

        function obj = generateMovie(obj, filename, speed)
            arguments
                obj
                filename string = "movie.mp4" % 保存するファイル名
                speed = 1       % 動画の再生速度
            end
            obj.makeMovie(@obj.moviePlot, obj.param.dt, obj.param.Nt, filename, speed);
        end

        function moviePlot(obj,t)
            % 動画用のフレーム単位プロット
            phase = obj.x(:,t)/obj.param.L*2*pi;    % エージェントの位相
            r_ = 1; % 描画円の半径
            plot(r_*cos(phase), r_*sin(phase),'*','Color','b');  % エージェント位置の表示
            hold on
            theta_ = 0:0.05:2*pi;
            plot(r_*cos(theta_), r_*sin(theta_),'LineWidth',0.8,'Color','k');    % 経路の表示
            xlim([-r_*1.2, r_*1.2]);
            ylim([-r_*1.2, r_*1.2]);
            hold off
            pbaspect([1 1 1])
        end

    end % methods
end % class