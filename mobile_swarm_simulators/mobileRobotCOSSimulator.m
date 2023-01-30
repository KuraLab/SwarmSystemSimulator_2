
classdef mobileRobotCOSSimulator < MobileRobots2dSimulator
     properties
        cos             % 結合振動子のインスタンス
     end

methods
    %%%%%%% 初期設定まわり %%%%%%%
    function obj = mobileRobotCOSSimulator()
        % コンストラクタ（宣言時に呼ばれる）
        obj@MobileRobots2dSimulator();    % 親クラスのコンストラクタも呼び出す
        obj = obj.setDefaultParameters();       % パラメタのデフォルト値を設定
        obj.cos = WaveInteractionSimulator();   % COS
    end

    function obj = setDefaultParameters(obj)
        obj = obj.setDefaultParameters@MobileRobots2dSimulator();   % スーパークラス側の読み出し
    end
    
    function obj = initializeVariables(obj)
        % 各種変数を初期化．シミュレーションをやり直す度に必ず呼ぶこと
        % 状態変数の定義と初期値の代入を行うこと
        obj = obj.initializeVariables@MobileRobots2dSimulator();   % スーパークラス側の読み出し
        obj.cos = obj.cos.setParam("phi_0",pi*rand(obj.param.Na,1));
        omega_0 = 1*ones(obj.param.Na,1);
        %omega_0(3) = 2;
        obj.cos = obj.cos.setParam("omega_0",omega_0);
    end
    
    function obj = defineSystem(obj)
        % システムの定義が必要な場合はここ．シミュレーションをやり直すたびに呼ぶこと
        %%% COSの各種設定 %%%
        obj.cos = obj.cos.setParam("dt",obj.param.dt);  % 基本設定を共有しておく
        obj.cos = obj.cos.setParam("Nt",obj.param.Nt);
        obj.cos = obj.cos.setParam("Na",obj.param.Na);
        obj.cos = obj.cos.initializeVariables();
    end

    %%%%%%%%%%%%%%%%%%%% 時間更新まわり %%%%%%%%%%%%%%%%%%

        function obj = simulate(obj)
            % シミュレーション本体
            disp("シミュレーションを開始します...")
            tic
            for t = 1:obj.param.Nt-1
                % ループ毎の更新をここに
                obj.G = obj.calcGraph(t);
                obj.cos = obj.cos.setGraph(obj.G);  % グラフを渡す
                obj.cos = obj.cos.setPosition(obj.x(:,:,t),t);
                obj.cos = obj.cos.stepSimulate(t);  % COS側の更新
                obj = obj.calcControlInput(t);  % 入力の計算
                obj.x(:,:,t+1) = obj.x(:,:,t) + obj.param.dt*obj.dxdt(:,:,t);   % オイラー法による更新
                obj.dxdt(:,:,t+1) = obj.dxdt(:,:,t) + obj.param.dt*obj.u(:,:,t);
            end % for
            toc
        end % simulate

        function obj = calcControlInput(obj,t)
            % 入力の生成．継承して使う
            arguments
                obj
                t    % 時刻
            end
            obj.u(:,:,t) = 0;  % 動くな
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
            %clim([0,1])
            colorbar
            text(obj.param.space_x(2)*0.7, obj.param.space_y(2)*0.8, "t = "+string(t), 'FontSize',12);
            hold off
        end

        function obj = generateMovie(obj, filename, speed)
            arguments
                obj
                filename string = "movie.mp4" % 保存するファイル名
                speed = 1       % 動画の再生速度
            end
            %obj.makeMovie(@obj.phasePlacePlot, obj.param.dt, obj.param.Nt, filename, speed, true);
            obj.makeMovie(@obj.edgeJudgePlot, obj.param.dt, obj.param.Nt, filename, speed, true);
        end

end
end% clasdef