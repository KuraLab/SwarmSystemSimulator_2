
classdef MobileRobots2dSimulator < Simulator
    % 時空間のシミュレータ
    properties
        % システムの変数を記載
        t_vec   % 固有時刻ベクトル
        x       % ロボット位置 [台数,空間次元,時刻]
        dxdt    % ロボット速さ [台数,空間次元,時刻]
        u       % 入力
        G       % グラフオブジェクト．MATLABのgraph参照
        wall     % 壁セグメントの集合 [開始/終了,空間次元,セグメント]
    end

    methods
        function obj = MobileRobots2dSimulator()
            % コンストラクタ（宣言時に呼ばれる）
            obj@Simulator();    % 親クラスのコンストラクタも呼び出す
            obj = obj.setDefaultParameters();       % パラメタのデフォルト値を設定
        end

    %%%%%%%%%%%%%% 初期設定まわり %%%%%%%%%%%%%

        function obj = setDefaultParameters(obj)
            % パラメータとデフォルト値を設定
            %%%%%%%% シミュレータの基本情報 %%%%%%%
            obj.param.dt = 0.05;    % 刻み時間
            obj.param.Nt = 400;    % 計算するカウント数
            obj.param.Na = 20;       % エージェント数
            obj.param.space_x = [-8 8]; % 空間のサイズを定義
            obj.param.space_y = [-8 8];
            %%%%%%%% システムパラメータ %%%%%%%%
            % obj.param.K = 1;       % ゲイン
            obj.param.rv = 1.6;      % 観測範囲
            %%%%%%%% 読み込みファイル名 %%%%%%%%
            obj.param.environment_file = "setting_files/environments/narrow_space.m";  % 環境ファイル
            obj.param.placement_file = "setting_files/init_conditions/narrow_20.m";    % 初期位置ファイル
            %%%%%%%%%%%%%% 初期値 %%%%%%%%%%%%%
            obj.param.x_0 = rand(obj.param.Na, 2);
            obj.param.dxdt_0 = zeros(obj.param.Na, 2);
        end
        
        function obj = initializeVariables(obj)
            % 各種変数を初期化．シミュレーションをやり直す度に必ず呼ぶこと
            % 状態変数の定義と初期値の代入を行うこと
            obj.t_vec = 0:obj.param.dt:obj.param.dt*(obj.param.Nt-1); % 時刻ベクトルの定義
            obj.x(:,:,:) = zeros(obj.param.Na, 2, obj.param.Nt);    % 状態変数の定義
            obj.x(:,:,1) = obj.param.x_0;   % 初期値の代入
            obj.dxdt(:,:,:) = zeros(obj.param.Na, 2, obj.param.Nt);    % 状態変数の定義
            obj.dxdt(:,:,1) = obj.param.dxdt_0;   % 初期値の代入
            obj.u(:,:,:) = zeros(obj.param.Na, 2, obj.param.Nt);    % 入力の履歴
        end
        
        function obj = readSettingFiles(obj)
            % 環境や初期配置の初期設定ファイルを読み込む
            % initializeVariablesの前に読み込む！！
            %%%%%% 初期配置の読み込み %%%%%%
            run(obj.param.placement_file);
            obj.param.Na = posset_Na;               % posset_Naに台数
            obj.param.x_0 = [posset_x, posset_y];   % posset_x[台数,1],posset_y[台数,1]に初期位置
            %%%%%% 環境情報の読み込み %%%%%%
            run(obj.param.environment_file);
            obj.param.space_x = [envset_xmin, envset_xmax]; % envset_xmin,maxにx方向の端の値を指定
            obj.param.space_y = [envset_ymin, envset_ymax];
            obj.wall = permute(envset_wall_segments,[1,2,3]); % 障害物セグメント情報を読み取り
        end

        function obj = defineSystem(obj)
            % システムの定義が必要な場合はここ．シミュレーションをやり直すたびに呼ぶこと
            
        end


    %%%%%%%%%%%%%%%%%%%% 時間更新まわり %%%%%%%%%%%%%%%%%%

        function obj = simulate(obj)
            % シミュレーション本体
            disp("シミュレーションを開始します...")
            tic
            for t = 1:obj.param.Nt-1
                % ループ毎の更新をここに
                obj.G = obj.calcGraph(t);
                obj = obj.calcControlInput(t);  % 入力の計算
                obj.x(:,:,t+1) = obj.x(:,:,t) + obj.param.dt*obj.dxdt(:,:,t);   % オイラー法による更新
                obj.dxdt(:,:,t+1) = obj.dxdt(:,:,t) + obj.param.dt*obj.u(:,:,t);
            end % for
            toc
        end % simulate
        
        function G_ = calcGraph(obj,t)
            % 所定時刻におけるグラフを更新
            arguments
                obj
                t   % 時刻
            end
            X = repmat(obj.x(:,1,t),1,obj.param.Na);    % x座標を並べた行列
            Y = repmat(obj.x(:,2,t),1,obj.param.Na);    % y座標を並べた行列
            distances = (X-X.').^2 + (Y-Y.').^2;  % ユークリッド距離の２乗．X-X.'でx座標の差分が得られる
            % 隣接行列はロボット間距離が観測範囲rvよりも小さいかどうか．対角要素は無視してグラフを作成
            G_ = graph(distances<obj.param.rv^2, 'omitselfloops');
        end

        function obj = calcControlInput(obj,t)
            % 入力の生成．継承して使う
            arguments
                obj
                t    % 時刻
            end
            disp("WARNIG : 継承前クラスのメソッドcalcControlInputが呼び出されている可能性があります")
            u_t = zeros(obj.param.Na, 2);   % 時刻tにおける入力
            %u_t(12,:) = [0.1,0.1];
            obj.u(:,:,t) = u_t;
        end

        function relative_vectors = calcVectorToWalls(obj,t)
            % 壁セグメントまでの相対位置
            % @return relative_vectors [ロボット数, 空間次元, 壁セグメント数]
            arguments
                obj
                t   % 時刻
            end
            Nwall_ = length(obj.wall); % 壁セグメントの数
            relative_vectors = zeros(obj.param.Na, 2, Nwall_);
            for p_ = 1:Nwall_   % セグメント毎に計算
                arg_ = atan2(obj.wall(2,2,p_)-obj.wall(1,2,p_),obj.wall(2,1,p_)-obj.wall(1,1,p_)); % セグメントのx軸からの偏角
                Rot_ = [cos(arg_), sin(arg_); -sin(arg_), cos(arg_)];   % 回転行列
                rotated_x_ = Rot_ * obj.x(:,:,t).'; % 回転後のエージェント座標．[空間次元, ロボット数]なので注意
                rotated_segment_ = Rot_ * obj.wall(:,:,p_).';  % 回転後のセグメント端点. y座標は同じになるはず．[空間次元, [開始/終了]]なので注意
                rv__ = zeros(2,obj.param.Na);   % 相対ベクトルを保持する中間変数．[空間次元,ロボット数]
                % 開始点のx座標よりもロボットのx座標が小さければ，開始点との相対位置
                rv__(:,:) = ( rotated_x_(1,:)<rotated_segment_(1,1) ).* (rotated_segment_(:,1)-rotated_x_(:,:));
                % 終了点のx座標よりもロボットのx座標が大きければ，終了点との相対位置
                rv__(:,:) = rv__(:,:) + ( rotated_x_(1,:)>rotated_segment_(1,2) ).* (rotated_segment_(:,2)-rotated_x_(:,:));
                % 終了点と開始点の間にロボットのx座標があるならば，y座標の差
                rv__(:,:) = rv__(:,:) + ( (rotated_x_(1,:)>=rotated_segment_(1,1)).*(rotated_x_(1,:)<=rotated_segment_(1,2)) ).*([rotated_x_(1,:);rotated_segment_(2,2)*ones(1,obj.param.Na)]-rotated_x_(:,:));
                
                relative_vectors(:,:,p_) = permute(Rot_.'*rv__,[2,1]);
            end
        end

    %%%%%%%%%%%%%%%%%%%%% 描画まわり %%%%%%%%%%%%%%%%%%

        function obj = placePlot(obj, t, view_edge, val)
            % ロボットの位置プロット
            arguments
                obj
                t               % 時刻
                view_edge = true                    % エッジ表示するか？
                val = zeros(obj.param.Na,1)         % ロボットに特徴づける値．これに応じてロボットの色が変わる
            end
            if view_edge    % エッジ表示するなら
                obj.showEdges(t);
            end
            obj.showWalls();    % 壁表示
            hold on
            scatter(obj.x(:,1,t),obj.x(:,2,t),120,val,'filled','MarkerEdgeColor','k'); % 散布図表示
            xlim(obj.param.space_x);    % 描画範囲を決定
            ylim(obj.param.space_y);
            pbaspect([1 1 1])             % 縦横のアスペクト比を合わせる
            colormap(gca,"cool")
            hold on
        end

        function obj = showEdges(obj,t)
            G_ = obj.calcGraph(t);  % グラフ計算
            e_ = table2array(G_.Edges);          % エッジ取得
            x_ = obj.x(:,1,t); 
            y_ = obj.x(:,2,t); 
            line(x_(e_).', y_(e_).','Color',"#0072BD",'LineWidth',1); % エッジの描画
        end
        
        function obj = showWalls(obj)
            % 壁の描画
            if isempty(obj.wall)    % 壁無かったら描画しない
                return
            end
            line(permute(obj.wall(:,1,:),[1,3,2]), permute(obj.wall(:,2,:),[1,3,2]), 'Color',"k",'LineWidth',1)
        end

        function obj = moviePlot(obj,t)
            % 動画用プロット
            delete(gca)
            obj.placePlot(t,true);
            text(obj.param.space_x(2)*0.7, obj.param.space_y(2)*0.8, "t = "+string(t), 'FontSize',12);
            hold off
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
            obj.makeMovie(@obj.moviePlot, obj.param.dt, obj.param.Nt, filename, speed, true);
        end

    end % methods
end