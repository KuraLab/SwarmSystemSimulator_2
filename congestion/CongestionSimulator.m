
% 1次元の渋滞シミュレータ

classdef CongestionSimulator< Simulator
    properties
        % システムの変数を記載
        t_vec   % 固有時刻ベクトル
        x       % 状態変数
        dxdt    % 1階微分
        u_history   % 入力履歴
        cbf     % CBF
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
            obj.param.Nt = 2000;    % 計算するカウント数
            obj.param.Na = 10;       % エージェント数
            %%%%%%%%%%%%% 環境情報 %%%%%%%%%%%%
            obj.param.L = 10;       % 周長
            %%%%%%%% システムパラメータ %%%%%%%%
            %obj.param.control_model = "sugiyama";   % 制御モデルの指定
            obj.param.control_model = "velocity_FB";   % 制御モデルの指定
            % 杉山モデル
            obj.param.k = 0;       % ゲイン
            obj.param.c_0 = 0;       % 初速度
            obj.param.r_s = obj.param.L/obj.param.Na*0.6;   % 等間隔の60%
            obj.param.a = 0;        % 時定数
            % 線形速度FBモデル
            obj.param.v_r = 0;      % 目標速度
            obj.param.k_v = 0;      % 追従ゲイン
            % 車間PD制御モデル
            obj.param.k_P = 0;      % 車間(P)ゲイン
            obj.param.k_D = 0;      % 速度差(D)ゲイン
            obj.param.r_r = 0;      % 目標車間
            % CBFパラメータ
            obj.param.CBF_enable = true;   % CBF使う？
            obj.param.T_c = 0.05;       % 時定数ゲイン(h0->h1)
            obj.param.gamma = 2;    % 粘性ゲイン（h1->h2）
            obj.param.devide = false;% 分散化するか？
            % 外乱
            obj.param.epsilon_x = 0;    % 初期位置ずらし量
            obj.param.epsilon_v_r = 0;    % 速度目標ずらし量
            obj.param.epsilon_v_0 = 0;    % 初期速度ずらし量
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
            obj.u_history(:,:) = zeros(obj.param.Na, obj.param.Nt);    % 状態変数の定義
            obj.x(:,1) = 0:obj.param.L/obj.param.Na:obj.param.L/obj.param.Na*(obj.param.Na-1);   % 等配分．摂動はこれに足しこむ形が良い
            obj.dxdt(:,1) = obj.param.c_0*ones(obj.param.Na,1);    % 初速度
            obj.x(1,1) = obj.x(1,1) + obj.param.epsilon_x * obj.param.L/obj.param.Na;   % 初期間隔のepsilon_x倍，あるエージェントをずらす
            obj.dxdt(1,1) = obj.dxdt(1,1) + obj.param.c_0*obj.param.epsilon_v_0;        % 初期速度への摂動
        end

        function obj = defineSystem(obj)
            % システムの定義が必要な場合はここ．シミュレーションをやり直すたびに呼ぶこと
            obj.cbf = CBF();
            obj.cbf = obj.cbf.setParameters(1,obj.param.r_s,obj.param.T_c,obj.param.gamma,obj.param.devide);    % CBFパラメータの設定

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
                if obj.param.CBF_enable == true % CBF
                    u = obj.applyCBF(u_nominal,t);
                else
                    u = u_nominal;  % CBF無ければそのまま
                end
                obj.u_history(:,t) = u; % 入力の記録
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
            Epsilon = ones(obj.param.Na,1);
            Epsilon(1) = Epsilon(1) + obj.param.epsilon_v_r;    % 摂動量を並べたベクトル
            u_nominal = obj.param.a*(V.*Epsilon-obj.dxdt(:,t));
        end

        function u_nominal = velocityFBController(obj, t)   % 速度FB，これだけだとぶつかる
            arguments
                obj
                t       % 時刻
            end
            Epsilon = ones(obj.param.Na,1);
            Epsilon(1) = Epsilon(1) + obj.param.epsilon_v_r;    % 摂動量を並べたベクトル
            u_nominal = obj.param.k_v*(obj.param.v_r.*Epsilon-obj.dxdt(:,t));    % 速度をvrに保つ
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

        function u = applyCBF(obj,u_nominal,t)
            arguments
                obj
                u_nominal   % ノミナル入力
                t           % 時刻
            end
            u = zeros(obj.param.Na,1);
            Delta_x = [obj.x(2:end,t);obj.x(1,t)]-obj.x(:,t);   % 前方エージェントとの差を返すNa*1ベクトル
            Delta_x = Delta_x + (Delta_x<0).*obj.param.L;       % 前方エージェントとの差が負になった場合． 周期条件を適用して正に戻す
            Delta_dxdt = [obj.dxdt(2:end,t);obj.dxdt(1,t)]-obj.dxdt(:,t);
            n_backs = [2:obj.param.Na,1];
            for n = 1:obj.param.Na
                obj.cbf = obj.cbf.addConstraints(Delta_x(n),Delta_dxdt(n)); % 前方との衝突回避
                if obj.param.devide == true        % 分散化する
                    obj.cbf = obj.cbf.addConstraints(-Delta_x(n_backs(n)),-Delta_dxdt(n_backs(n))); % 後方との衝突回避
                end
                u(n) = obj.cbf.apply(u_nominal(n)); % 2次計画法を解く
                obj.cbf = obj.cbf.clearConstraints();   % 制約削除
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%% 描画 %%%%%%%%

        function obj = positionLinearPlot(obj)
            % 基本プロット
            disp("描画を開始します...")

            figure  % 生値プロット
            plot(obj.t_vec, obj.x(:,:));
            title("エージェント位置の時間変化")
            xlabel("時刻 t [s]")
            ylabel("位置 x_i")
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
            ylabel("前方エージェントとの相対位置 x_{ij}")
            legend(string(1:obj.param.Na))
            ylim([0, obj.param.L])
            title(obj.titleSet(),'Interpreter','latex')
        end

        function title_ = titleSet(obj) % 図のタイトルにパラメータを入れる場合
            if obj.param.control_model == "sugiyama"
                title_ = "$k/a = "+string(obj.param.k/obj.param.a)+",r_s = "+string(obj.param.r_s)+",\epsilon_x = "+string(obj.param.epsilon_x)+",\epsilon_{v_0} = "+string(obj.param.epsilon_v_0)+",\epsilon_{v_r} = "+string(obj.param.epsilon_v_r)+"$";
            elseif obj.param.control_model == "velocity_FB"
                title_ = "$v_r = "+string(obj.param.v_r)+",k_v = "+string(obj.param.k_v)+",\epsilon_x = "+string(obj.param.epsilon_x)+",\epsilon_{v_0} = "+string(obj.param.epsilon_v_0)+",\epsilon_{v_r} = "+string(obj.param.epsilon_v_r)+"$";
            elseif obj.param.control_model == "PD"
                title_ = "$k_P = "+string(obj.param.k_P)+",k_D = "+string(obj.param.k_D)+",r_r = "+string(obj.param.r_r)+",\epsilon_x = "+string(obj.param.epsilon_x)+",\epsilon_{v_0} = "+string(obj.param.epsilon_v_0)+"$";
            end
            if obj.param.CBF_enable == true
                title_ = title_ + " $,\gamma = "+string(obj.param.gamma)+",r_s = "+string(obj.param.r_s)+"$";
            end
        end

        function obj = plotSafetyFunctionOnPhase(obj, pair)
            arguments
                obj
                pair = [1,2]   % 描画するエージェントのペア
            end
            figure
            x_ij = obj.x(pair(2),1:end-1)-obj.x(pair(1),1:end-1);   % 相対位置
            x_ij = x_ij + (x_ij<0).*obj.param.L;       % 前方エージェントとの差が負になった場合． 周期条件を適用して正に戻す
            dotx_ij = obj.dxdt(pair(2),1:end-1)-obj.dxdt(pair(1),1:end-1);  % 相対速度
            ddotx_ij = obj.u_history(pair(2),:)-obj.u_history(pair(1),:);   % 相対加速度
            h = x_ij - obj.param.r_s + obj.param.T_c * dotx_ij;
            doth = dotx_ij + obj.param.T_c * ddotx_ij;
            
            plot(h(1:end-1), doth(2:end));
            %plot(h(:), doth(:));
            hold on
            h_vec = 0:0.1:1.2;
            plot(h_vec, -obj.param.gamma*h_vec);
            xlim([0,1.2]);
            ylim([-2,0.6])
            legend("$h_{12}$","$\dot{h}=-\gamma h$",'Interpreter','latex');
            xlabel("$h$",'Interpreter','latex')
            ylabel("$\dot{h}$",'Interpreter','latex')
            title(obj.titleSet(),'Interpreter','latex')
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
            plot(r_*cos(phase(2:end)), r_*sin(phase(2:end)),'*','Color','b');  % エージェント位置の表示
            hold on
            plot(r_*cos(phase(1)), r_*sin(phase(1)),'*','Color','r');  % エージェント位置の表示
            theta_ = 0:0.05:2*pi;
            plot(r_*cos(theta_), r_*sin(theta_),'LineWidth',0.8,'Color','k');    % 経路の表示
            xlim([-r_*1.2, r_*1.2]);
            ylim([-r_*1.2, r_*1.2]);
            hold off
            pbaspect([1 1 1])
        end

    end % methods
end % class