
classdef AttractantFieldGenerator < Simulator
    % 誘引源をつくることが目的
    properties
        % システムの変数を記載
        t_vec   % 固有時刻ベクトル
        x_vec   % x方向の座標
        y_vec   % y方向の座標
        c       % 濃度
        wall    % 壁セグメントの集合 [開始/終了,空間次元,セグメント]
        active_area     % 領域内なら1, 領域外なら0
        source_index    % 匂い源の位置（インデックス）
    end

    methods
        function obj = AttractantFieldGenerator()
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
            obj.param.dx = 0.5;       % 空間の刻み長さ
            obj.param.space_x = [-8 8]; % 空間のサイズを定義
            obj.param.space_y = [-8 8];
            %%%%%%%% システムパラメータ %%%%%%%%
            obj.param.D = 1;       % 拡散定数
            obj.param.source_pos = [6 3];   % 匂い源の場所
            obj.param.intense = 1;          % 匂い源強さ 
            obj.param.upwall_segment = [];    % 上限を決めるセグメントの場所（枠除く）
            %%%%%%%% 読み込みファイル名 %%%%%%%%
            obj.param.environment_file = "setting_files/environments/narrow_space.m";  % 環境ファイル
            %%%%%%%%%%%%%% 初期値 %%%%%%%%%%%%%
            %obj.param.c_0 = zeros(obj.param.Na, 2);
        end
        
        function obj = initializeVariables(obj)
            % 各種変数を初期化．シミュレーションをやり直す度に必ず呼ぶこと
            % 状態変数の定義と初期値の代入を行うこと
            obj.t_vec = 0:obj.param.dt:obj.param.dt*(obj.param.Nt-1); % 時刻ベクトルの定義
            obj.x_vec = obj.param.space_x(1):obj.param.dx:obj.param.space_x(2);
            obj.y_vec = obj.param.space_y(1):obj.param.dx:obj.param.space_y(2);
            obj.c(:,:,:) = zeros(length(obj.x_vec), length(obj.y_vec), obj.param.Nt);    % 状態変数の定義
            obj.c(:,:,1) = zeros(length(obj.x_vec), length(obj.y_vec));   % 初期値の代入
            obj.active_area = ones(length(obj.x_vec), length(obj.y_vec));   % 最初は全部領域内
            obj.source_index = round( (obj.param.source_pos-[obj.param.space_x(1) obj.param.space_y(1)])/obj.param.dx ) + 1;    % 匂い源インデックスの計算
        end
        
        function obj = readSettingFiles(obj)
            % 環境や初期配置の初期設定ファイルを読み込む
            % initializeVariablesの前に読み込む！！
            %%%%%% 環境情報の読み込み %%%%%%
            run(obj.param.environment_file);
            obj.param.space_x = [envset_xmin, envset_xmax]; % envset_xmin,maxにx方向の端の値を指定
            obj.param.space_y = [envset_ymin, envset_ymax];
            obj.wall = permute(envset_wall_segments,[1,2,3]); % 障害物セグメント情報を読み取り
            obj.param.upwall_segment = envset_upwall;
        end

        function obj = defineSystem(obj)
            % システムの定義が必要な場合はここ．シミュレーションをやり直すたびに呼ぶこと
            obj = obj.calcActiveArea(); % active_areaの決定
        end

        function obj = calcActiveArea(obj)
            % 領域の外か内か判定する関数
            % 本当はもっと一般にすべきだが，上限を決める壁を決めた上で，x軸対象という強い仮定を置く
            for nx_ = 1:length(obj.x_vec)  % x軸方向で走らせる(添え字)
                x_ = obj.x_vec(nx_);        % その添え字でのx座標
                for ws_ = obj.param.upwall_segment  % 上限セグメントで走らせる
                    if (x_<obj.wall(1,1,ws_)||x_>obj.wall(2,1,ws_)) % 選択中のセグメントの担当範囲か？
                        continue
                    end
                    x1 = obj.wall(1,1,ws_); y1 = obj.wall(1,2,ws_); % セグメントの始点と終点
                    x2 = obj.wall(2,1,ws_); y2 = obj.wall(2,2,ws_);
                    m_ = (-x2+x_)/(x1-x2); % 比の計算
                    y_ = m_*y1 + (1-m_)*y2; % 線分上のyの値
                    obj.active_area(nx_, abs(obj.y_vec)>=abs(y_)) = 0;  % 線分上のyの値より絶対値が小さい所のみ有効．下側も対称ならこれでよし
                end
            end
            obj.active_area(1,:) = 0;   % 4方の果ては全てエリア外
            obj.active_area(end,:) = 0;
            obj.active_area(:,1) = 0;
            obj.active_area(:,end) = 0;
        end

        %%%%%%%%%%%%%%%%%%%% 時間更新まわり %%%%%%%%%%%%%%%%%%

        function obj = simulate(obj)
            % シミュレーション本体
            disp("シミュレーションを開始します...")
            tic
            for t = 1:obj.param.Nt-1
                % ループ毎の更新をここに
                obj.c(obj.source_index(1),obj.source_index(2),t) = obj.param.intense;   % 匂い源

                L_ = zeros(length(obj.x_vec),length(obj.y_vec));    % 2階の空間差分
                L_(2:end-1,2:end-1) = obj.c(1:end-2,2:end-1,t) + obj.c(3:end,2:end-1,t) + obj.c(2:end-1,1:end-2,t) + obj.c(2:end-1,3:end,t) - 4*obj.c(2:end-1,2:end-1,t);
                L_ = L_/obj.param.dx^2;
                % (左+右+上+下-4自分)/(dx)^2．外枠部分は0のまま
                obj.c(:,:,t+1) = obj.c(:,:,t) + obj.param.dt*obj.param.D*L_;    % 拡散eq
                obj.c(:,:,t+1) = obj.c(:,:,t+1).*obj.active_area;  % アクティブエリア外はいらぬ 
            end % for
            toc
        end % simulate

            %%%%%%%%%%%%%%%%%%%%% 描画まわり %%%%%%%%%%%%%%%%%%

        function obj = plot(obj, t)
            % ロボットの位置プロット
            arguments
                obj
                t               % 時刻
            end
            
            imagesc(obj.param.space_x, obj.param.space_y, log(obj.c(:,:,t).'));
            obj.showWalls();    % 壁表示
            xlim(obj.param.space_x);    % 描画範囲を決定
            ylim(obj.param.space_y);
            axis xy
            pbaspect([1 1 1])             % 縦横のアスペクト比を合わせる
            %colormap(gca,"cool")
            c_ = colorbar;
            c_.Label.String = "c(x)";
            xlabel("x")
            ylabel("y")
            hold on
        end

        function obj = showWalls(obj)
            % 壁の描画
            if isempty(obj.wall)    % 壁無かったら描画しない
                return
            end
            line(permute(obj.wall(:,1,:),[1,3,2]), permute(obj.wall(:,2,:),[1,3,2]), 'Color',"k",'LineWidth',1)
        end

    end % method
end % classdef