
classdef SwarmWithWaveInteractionSimulation < MobileRobots2dSimulator
    
    properties
        cbf             % CBFのインスタンス
        attract_field   % 誘導場生成クラスのインスタンス
        cos             % 結合振動子のインスタンス
        is_connected = true     % グラフが連結かどうか
    end

    methods
        %%%%%%% 初期設定まわり %%%%%%%
        function obj = SwarmWithWaveInteractionSimulation()
            % コンストラクタ（宣言時に呼ばれる）
            obj@MobileRobots2dSimulator();    % 親クラスのコンストラクタも呼び出す
            obj = obj.setDefaultParameters();       % パラメタのデフォルト値を設定
            obj.cbf = CollisionAvoidanceCBF();              % CBF
            obj.attract_field = AttractantFieldGenerator(); % 誘導場生成クラス
            obj.cos = WaveInteractionSimulator();   % COS
        end

        function obj = setDefaultParameters(obj)
            obj = obj.setDefaultParameters@MobileRobots2dSimulator();   % スーパークラス側の読み出し
            obj.param.kp = 1;   % 勾配追従力ゲイン
            obj.param.kf = 1;   % 群形成力ゲイン
            obj.param.rc = 1;   % 群形成力の平衡距離
            obj.param.kd = 1;   % 粘性項
            obj.param.attract_force_type = "field_xy";% x方向のみ誘導力の形式
            obj.param.is_debug_view = false;    % デバッグ表示をするか？

        end
        
        function obj = initializeVariables(obj)
            % 各種変数を初期化．シミュレーションをやり直す度に必ず呼ぶこと
            % 状態変数の定義と初期値の代入を行うこと
            obj = obj.initializeVariables@MobileRobots2dSimulator();   % スーパークラス側の読み出し
            obj.cos = obj.cos.setParam("phi_0",rand(obj.param.Na,1));
        end
        
        function obj = defineSystem(obj)
            % システムの定義が必要な場合はここ．シミュレーションをやり直すたびに呼ぶこと
            %%% 誘導場をつくる %%%
            obj.attract_field = obj.attract_field.setParam("environment_file",obj.param.environment_file);  % 環境情報を誘導場クラスに渡す
            obj.attract_field = obj.attract_field.readSettingFiles();   % 設定ファイル読み出し
            obj.attract_field = obj.attract_field.initializeVariables();
            obj.attract_field = obj.attract_field.defineSystem();
            obj.attract_field = obj.attract_field.simulate();           % simulateまで回す．以降，obj.attract_field.c(:,:,end)に誘導場情報がある
            %%% COSの各種設定 %%%
            obj.cos = obj.cos.setParam("dt",obj.param.dt);  % 基本設定を共有しておく
            obj.cos = obj.cos.setParam("Nt",obj.param.Nt);
            obj.cos = obj.cos.setParam("Na",obj.param.Na);
            obj.cos = obj.cos.setParam("interaction_type","wave");
            obj.cos = obj.cos.initializeVariables();
        end

        %%%%%%%% 時間更新 %%%%%%%%%
        function obj = calcControlInput(obj,t)
            % 入力の生成．継承して使う
            arguments
                obj
                t    % 時刻
            end
            obj.showSimulationTime(t);
            u_t = zeros(obj.param.Na, 2);   % 時刻tにおける入力
            u_nominal = zeros(obj.param.Na, 2); % CBFをかける前のノミナル入力
            Adj = full(adjacency(obj.G));   % 隣接行列
            %%%% 勾配追従力 %%%%
            u_p = zeros(obj.param.Na, 2);
            pos_index = round( (obj.x(:,:,t)-repmat([obj.param.space_x(1) obj.param.space_y(1)],obj.param.Na,1))./obj.attract_field.param.dx ) + 1;
            % ロボットのポジションインデックスの計算 round( (x-x_min)/dx )+1 結果は[エージェント数,空間次元]
            for i = 1:obj.param.Na
                %u_p(i,:) = obj.param.kp*( obj.attract_field.cx(pos_index(i,1), pos_index(i,2))*[1 0] + obj.attract_field.cy(pos_index(i,1), pos_index(i,2))*[0 1] );
                cx = obj.attract_field.cx(pos_index(i,1), pos_index(i,2));
                cy = obj.attract_field.cy(pos_index(i,1), pos_index(i,2));
                if obj.param.attract_force_type == "field_xonly"
                    u_p(i,:) = obj.param.kp*( cx*[1 0] )/norm([cx,cy]);   % 誘導場をx方向のみ利用
                elseif obj.param.attract_force_type == "field_xy"
                    u_p(i,:) = obj.param.kp*( cx*[1 0] + cy*[0 1] )/norm([cx,cy]);  % 誘導場を利用
                elseif obj.param.attract_force_type == "linear_fbx"
                    u_p(i,:) = obj.param.kp*( [1 0] );  % ただのx方向への単位ベクトル
                end
                % u_p = kp( c_x ex + c_y ey )
            end
            %%%% 群形成力 %%%%
            u_f = zeros(obj.param.Na, 2);   % 群形成力
            X = repmat(obj.x(:,1,t),1,obj.param.Na);  % 位置x
            Y = repmat(obj.x(:,2,t),1,obj.param.Na);  % 位置y
            X_ij = X.'-X;   % 相対位置 X(i,j) = x(j)-x(i)
            Y_ij = Y.'-Y;
            D_ij = sqrt(X_ij.^2+Y_ij.^2)/obj.param.rc + eye(obj.param.Na);   % 正規化相対距離 (零割防止のため対角に1がならぶ)
            u_f(:,1) = obj.param.kf*sum( full(adjacency(obj.G)).*(-X_ij/obj.param.rc).*(D_ij.^(-3)-D_ij.^(-2)).*exp(-D_ij) ,2);
            u_f(:,2) = obj.param.kf*sum( full(adjacency(obj.G)).*(-Y_ij/obj.param.rc).*(D_ij.^(-3)-D_ij.^(-2)).*exp(-D_ij) ,2);
            %%%% 位相勾配の利用 %%%%
            obj.cos = obj.cos.setGraph(obj.G);  % グラフを渡す
            obj.cos = obj.cos.setPosition(obj.x(:,:,t),t);
            obj.cos = obj.cos.stepSimulate(t);  % COS側の更新

            % ノミナル入力の決定
            u_nominal = u_p + u_f;

            %%%% CBF %%%%
            % 詳細はCollisionAvoidanceCBF.mを参照
            x_io = obj.calcVectorToWalls(t);    % 壁との相対位置ベクトル
            for i = 1:obj.param.Na
                % ロボット間衝突回避CBF %
                obj.cbf = obj.cbf.setParameters(1,1,obj.param.dt,1,true);
                x_ij = obj.x(:,:,t) - obj.x(i,:,t);          % 相対位置ベクトル
                dxdt_ij = obj.dxdt(:,:,t) - obj.dxdt(i,:,t); % 相対速度ベクトル
                obj.cbf = obj.cbf.addConstraints([x_ij(Adj(:,i)==1,1), x_ij(Adj(:,i)==1,2)], [dxdt_ij(Adj(:,i)==1,1), dxdt_ij(Adj(:,i)==1,2)]);
                % 隣接ロボットとの相対ベクトルのみCBF制約として利用
                % 壁との衝突回避CBF %
                obj.cbf = obj.cbf.setParameters(1,0.5,obj.param.dt,1,false);
                obj.cbf = obj.cbf.addConstraints(permute(x_io(i,:,:),[3,2,1]), -repmat(obj.dxdt(i,:,t),length(x_io(i,:,:)),1));
                % 壁との相対位置ベクトルと，自身の速度ベクトル(壁との相対速度ベクトル)をCBFに入れる
                % CBFの適用 %
                u_t(i,:) = obj.cbf.apply(u_nominal(i,:));
                obj.cbf = obj.cbf.clearConstraints();
            end
            %{
            obj.cbf = obj.cbf.setParameters(1,1,obj.param.dt,1,true);
            % 1番用CBF
            obj.cbf = obj.cbf.addConstraints(obj.x(2,:,t)-obj.x(1,:,t), obj.dxdt(2,:,t)-obj.dxdt(1,:,t));
            u_t(1,:) = obj.cbf.apply(u_nominal(1,:));
            obj.cbf = obj.cbf.clearConstraints();
            % 2番用CBF
            obj.cbf = obj.cbf.addConstraints(obj.x(1,:,t)-obj.x(2,:,t), obj.dxdt(1,:,t)-obj.dxdt(2,:,t));
            u_t(2,:) = obj.cbf.apply(u_nominal(2,:));
            obj.cbf = obj.cbf.clearConstraints();
            %}
            %u_t(12,:) = [0.1,0.1];

            %%%% 最終的な入力の生成 %%%%
            obj.u(:,:,t) = u_t - obj.param.kd*obj.dxdt(:,:,t);  % CBF後に粘性が入っている…

            %%%% デバッグ %%%%
            if obj.param.is_debug_view
                figure
                obj.placePlot(t,true);
                hold on
                quiver(obj.x(:,1,t),obj.x(:,2,t),u_p(:,1),u_p(:,2));    % 勾配追従力プロット
            end
            %%%% 連結性の判定 %%%%
            Lap_ = full(laplacian(obj.G));  % グラフラプラシアン
            [~,Sigma] = eig(Lap_);  % 固有値展開
            sigma_ = sort(diag(Sigma));
            if (sigma_(2) <= 10^-5)
                obj.is_connected = false;
            end
        end

            %%%%%%%%%%%%% 描画まわり %%%%%%%%%%%%%%%
        function obj = phasePlacePlot(obj, t)
            % ロボットの位置プロット
            arguments
                obj
                t               % 時刻
            end
            delete(gca)
            obj = obj.placePlot(t,true, sin( obj.cos.phi(:,1,t) ) );
            clim([-1,1])
            colorbar
            text(obj.param.space_x(2)*0.7, obj.param.space_y(2)*0.8, "t = "+string(t), 'FontSize',12);
            hold off
        end

        function obj = edgeJudgePlot(obj, t, dim)
            % ロボットの位置プロット
            arguments
                obj
                t               % 時刻
                dim = 2         % 次元
            end
            delete(gca)
            obj = obj.placePlot(t,true, obj.cos.is_edge(:,dim,t));
            clim([0,1])
            colorbar
            text(obj.param.space_x(2)*0.7, obj.param.space_y(2)*0.8, "t = "+string(t), 'FontSize',12);
            hold off
        end

        function obj = generateMovieEstimate(obj, filename, speed)
            % 相対位置判定のムービーを生成
            arguments
                obj
                filename string = "movie.mp4" % 保存するファイル名
                speed = 1       % 動画の再生速度
            end
             obj.makeMovie(@obj.edgeJudgePlot, obj.param.dt, obj.param.Nt, filename, speed, true);
        end

        function obj = generateMovie(obj, filename, speed)
            arguments
                obj
                filename string = "movie.mp4" % 保存するファイル名
                speed = 1       % 動画の再生速度
            end
            obj.makeMovie(@obj.phasePlacePlot, obj.param.dt, obj.param.Nt, filename, speed, true);
        end
        
    end

end % clasdef