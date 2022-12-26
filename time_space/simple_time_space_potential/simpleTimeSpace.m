classdef simpleTimeSpace < Simulator
% 時空間のシミュレータ
    properties
        tau_vec   % 固有時刻ベクトル
        q         % 時空間座標
        dqdtau    % 時空間座標の一階微分
        tspace    % 時空間の形
    end

    methods
        function obj = simpleTimeSpace()
            obj@Simulator();
            obj = obj.setDefaultParameters();
        end

        function obj = setDefaultParameters(obj)
            % パラメータとデフォルト値を設定
            %%%%%%%% シミュレータの基本情報 %%%%%%%
            obj.param.dtau = 0.05;    % 刻み時間
            obj.param.Ntau = 400;    % 計算するカウント数
            obj.param.tsp_dim = 2;      % 時空間の次元
            %%%%%%%% システムパラメータ %%%%%%%%
            obj.param.metric = "const_input";    % 計量の形
            obj.param.A = 2;    % 定数1
            obj.param.B = 1;    % 定数2
            obj.param.C = 1;    % 定数3
            obj.param.f = 1;    % 入力
            obj.param.c = 100;  % 光の速さにあたる定数
            %%%%%%%%%%%%%% 初期値 %%%%%%%%%%%%%
            obj.param.q_0 = [0; 0];
            obj.param.dqdtau_0 = [obj.param.c, 0];
        end
        
        function obj = initializeVariables(obj)
            % 各種変数を初期化．シミュレーションをやり直す度に必ず呼ぶこと
            % 状態変数の定義と初期値の代入を行うこと
            obj.tau_vec = 0:obj.param.dtau:obj.param.dtau*(obj.param.Ntau-1); % 時刻ベクトルの定義
            obj.q = zeros(obj.param.tsp_dim, obj.param.Ntau);   % 状態変数phiの定義
            obj.dqdtau = zeros(obj.param.tsp_dim, obj.param.Ntau);
            obj.q(:,1) = obj.param.q_0;                % 初期値の代入
            obj.dqdtau(:,1) = obj.param.dqdtau_0;
        end

        function obj = defineSystem(obj)
            % システムの定義が必要な場合はここ．シミュレーションをやり直すたびに呼ぶこと
            if (obj.param.metric == "const_input")
                % 定数入力
                obj.tspace = constantInputMetric(obj.param.tsp_dim);
                obj.tspace = obj.tspace.setMetric(obj.param.A, obj.param.B, obj.param.C, obj.param.f, obj.param.c);
            end
            if (obj.param.metric == "deSitter")
                % ドジッター空間に入力
                obj.tspace = deSitterMetric(obj.param.tsp_dim);
                obj.tspace = obj.tspace.setMetric(obj.param.f, obj.param.c);
            end
        end

        function obj = simulate(obj)
            % シミュレーション本体
            disp("シミュレーションを開始します...")
            tic
            for tau = 1:obj.param.Ntau-1
                Gamma2 = obj.tspace.calcChristoffelNumWithoutpartialG(obj.q(:,tau));
                obj.q(:,tau+1) = obj.q(:,tau) + obj.param.dtau*obj.dqdtau(:,tau);
                dxdtau = obj.dqdtau(:,tau)*obj.dqdtau(:,tau).'; % テンソル積の計算を楽にするために，行列にしておく
                dx2dtau2 = -tensorprod(Gamma2,dxdtau,[1 2],[1 2])+[0;obj.param.f]; % テンソル籍の計算
                obj.dqdtau(:,tau+1) = obj.dqdtau(:,tau) + obj.param.dtau*dx2dtau2;
            end % for
            toc
        end % simulate

        function obj = plot(obj)
            % 各種プロット
            disp("描画を開始します...")

            figure  % 生値プロット
            plot(obj.tau_vec, obj.q(:,:));
            title("状態の固有時間変化 (c="+string(obj.param.c)+", dx/d\tau(0) = ["+string(obj.param.dqdtau_0(1))+", "+string(obj.param.dqdtau_0(2))+"])")
            xlabel("固有時刻 \tau [s]")
            legend(["x^0" "x^1"])

            figure;  % 生値プロット
            plot(obj.q(1,:)/obj.param.c, obj.q(2,:));
            title("状態の世界時間変化 (c="+string(obj.param.c)+", dx/d\tau(0) = ["+string(obj.param.dqdtau_0(1))+", "+string(obj.param.dqdtau_0(2))+"])")
            ylabel("x^1")
            xlabel("時刻 t [s]")
            %legend(string(1:obj.param.Na))
        end

        function obj = subplot(obj)
            % 状態の世界時間プロット
            plot(obj.q(1,:)/obj.param.c, obj.q(2,:));
            %legend(string(1:obj.param.Na))
        end

        function obj = subplot_eigentime(obj,dim)
            % 状態の固有時間プロット
            arguments
                obj
                dim % プロットする次元
            end
            plot(obj.tau_vec, obj.q(dim,:));
            %legend(string(1:obj.param.Na))
        end

        function obj = showEigen(obj, vecdim1, vecdim2)
            % 固有値と固有ベクトルをグリッドの各点毎に表示
            arguments
                obj
                vecdim1 % 値を計算するグリッド点のdim1方向の集合
                vecdim2 % 値を計算するグリッド点のdim2方向の集合
            end
            %figure
            v1 = zeros(2,length(vecdim1)*length(vecdim2));   % ベクトルの集合
            v2 = zeros(2,length(vecdim1)*length(vecdim2));
            x1 = zeros(2,length(vecdim1)*length(vecdim2));   % 位置の集合
            for n1 = 1:length(vecdim1)
                for n2 = 1:length(vecdim2)
                    G = obj.tspace.calcMetricNum([obj.param.c*vecdim1(n1);vecdim2(n2)]);
                    [V,D] = eig(G);
                    v1(:,(n1-1)*length(vecdim2)+(n2-1)+1) = D(1,1)*V(:,1);
                    v2(:,(n1-1)*length(vecdim2)+(n2-1)+1) = D(2,2)*V(:,2);
                    x1(:,(n1-1)*length(vecdim2)+(n2-1)+1) = [vecdim1(n1);vecdim2(n2)];
                end
            end
            quiver(x1(1,:),x1(2,:),v1(1,:),v1(2,:),'LineWidth',1.5);
            hold on
            quiver(x1(1,:),x1(2,:),v2(1,:),v2(2,:),'LineWidth',1.5);
        end

    end % methods
end % class