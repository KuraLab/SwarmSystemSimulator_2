classdef symbolicMetricSpace
% 計量付き空間のシンボリック表現
    properties(Access = protected)
        r_M         % シュヴァルツシルト半径
        dim         % 時空間の次元
        x           % 時空間の座標
        G           % 計量行列
        Ginv        % 計量の逆行列
        partialG    % 計量の偏微分
        Gamma1      % 第1種クリストフェル記号
        Gamma2      % 第2種クリストフェル記号
    end

    methods (Access = public)

        function obj = symbolicMetricSpace(dim) % コンストラクタ
            arguments
                dim {mustBeInteger} % 時空間の次元
            end
            % 定義諸々
            obj.dim = dim;
            obj.x = sym('x%d',[dim,1]);
            obj.G = sym('g_%d%d',[dim,dim]);
            obj.partialG = sym('dg_%d%d_%d',[dim,dim,dim]);
            obj.Gamma1 = sym('Gamma_%d%d_%d',[dim,dim,dim]);
            obj.Gamma2 = sym('Gamma_%d%d_%d',[dim,dim,dim]);
        end


        % 計量行列の設定
        function obj = setMetric(obj,G)     % 任意の計量行列を設定する．現状，外からxのシンボリック表現が見えないので，使えない？
            arguments
                obj
                G % 計量行列のシンボリック表現
            end
            obj.G = G;
            % 各量の計算
            obj = obj.calcSymbolicGeometry();
        end

        function obj = setPointPotentialMetric(obj,r_M,center_point) % 点源ポテンシャル
            arguments
                obj
                r_M = 0.01                          % シュヴァルツシルト半径．各種定数を押し込めて使っている．
                center_point = zeros(obj.dim-1,1)   % 点ポテンシャル源の位置
            end
            obj.r_M = r_M;
            r = sqrt(sum((obj.x(2:obj.dim,1)-center_point).^2));
            obj.G(1,1) = 1-obj.r_M/r;
            obj.G(2:obj.dim,1) = 0;
            obj.G(1,2:obj.dim) = 0;
            for alpha = 2:obj.dim
                for beta = 2:obj.dim
                    obj.G(alpha,beta) = -1*(alpha==beta)-obj.r_M*(obj.x(alpha,1)-center_point(alpha-1,1))*(obj.x(beta,1)-center_point(beta-1,1))/r^3;
                end
            end
            obj = obj.calcSymbolicGeometry();
            % disp(obj.G)
        end

        function obj = setMultiPointPotentialMetric(obj,r_M,center_points) % 静止した点ポテンシャルが複数
            arguments
                obj
                r_M                       % シュヴァルツシルト半径．各種定数を押し込めて使っている．1×点数
                center_points              % 点ポテンシャル源の位置 次元×点数
            end
            obj.r_M = r_M;
            num_point = length(r_M);    % 点の数
            r = sqrt(sum((obj.x(2:obj.dim,1)-center_points).^2,1));
            obj.G(1,1) = 1-sum(obj.r_M./r);
            obj.G(2:obj.dim,1) = 0;     % 時不変
            obj.G(1,2:obj.dim) = 0;
            for alpha = 2:obj.dim
                for beta = 2:obj.dim
                    obj.G(alpha,beta) = -1*(alpha==beta)-sum(obj.r_M.*(obj.x(alpha,1)-center_points(alpha-1,:)).*(obj.x(beta,1)-center_points(beta-1,:))./r.^3);
                    % 各点に対してr_M*x^\alpha x^beta/r^3 を計算したものが列ベクトルに入って，その行和を取ってスカラーに
                end
            end
            obj = obj.calcSymbolicGeometry();
        end

        function obj = setInteractionPotentialMetric(obj,r_M)   % 相互作用型のポテンシャル
            % とりあえず次元6で固定
            arguments
                obj
                r_M                 % シュヴァルツシルト半径
            end
            obj.r_M = r_M;
            r = sqrt(sum((obj.x(2:3,1)-obj.x(5:6,1)).^2,1));    % エージェント間距離に変更
            obj.G(1,1) = 1-sum(obj.r_M./r);
            obj.G(2:6,1) = 0;     % 時不変
            obj.G(1,2:6) = 0;
            obj.G(2:6,4) = 0;     % 時不変
            obj.G(4,2:6) = 0;
            obj.G(4,4) = 1-sum(obj.r_M./r);
            r_vec = sym("r_vec_%d%d",[6,1]);
            r_vec(2) = obj.x(2,1)-obj.x(5,1);
            r_vec(3) = obj.x(3,1)-obj.x(6,1);
            r_vec(5) = obj.x(5,1)-obj.x(2,1);
            r_vec(6) = obj.x(6,1)-obj.x(3,1);
            for alpha = [2,3,5,6]
                for beta = [2,3,5,6]
                    obj.G(alpha,beta) = -1*(alpha==beta)-sum(obj.r_M.*(r_vec(alpha,1)).*(r_vec(beta,1))./r.^3);
                    % 各点に対してr_M*x^\alpha x^beta/r^3 を計算したものが列ベクトルに入って，その行和を取ってスカラーに
                end
            end
            obj = obj.calcSymbolicGeometry();
        end

        function obj = setAllPointPotential(obj,r_M_interaction,r_size_interaction,r_M_point,r_size_point,center_points)   % 相互作用+静止点ポテンシャル
            arguments
                obj
                r_M_interaction % 相互作用のシュバルツシルト半径
                r_size_interaction % エージェント半径
                r_M_point       % シュヴァルツシルト半径．各種定数を押し込めて使っている．1×点数
                r_size_point          % 質点の半径．1×点数
                center_points   % 点ポテンシャル源の位置 次元×点数
            end
            r_vec = sym("r_vec_%d%d",[6,length(r_M_point)+1]);    % 相互作用ないし静止点に対する相対位置ベクトル, 6×(点数+1)
            r_M = [r_M_interaction, r_M_point];                   % 相互作用も静止点も一気に計算する
            r_size = [r_size_interaction, r_size_point];
            obj.r_M = r_M;
            % 相互作用の位置ベクトル
            r_vec(1,:) = 0;
            r_vec(4,:) = 0;
            r_vec(2,1) = obj.x(2,1)-obj.x(5,1);
            r_vec(3,1) = obj.x(3,1)-obj.x(6,1);
            r_vec(5,1) = obj.x(5,1)-obj.x(2,1);
            r_vec(6,1) = obj.x(6,1)-obj.x(3,1);
            % 静止点の位置ベクトル
            r_vec(2:3,2:end) = obj.x(2:3,1)-center_points(:,:);
            r_vec(5:6,2:end) = obj.x(5:6,1)-center_points(:,:);
            r = [sqrt(sum(r_vec(1:3,:).^2,1))-r_size; sqrt(sum(r_vec(4:6,:).^2,1))-r_size];   % 距離．2×(点数+1)行列で，1行目がx^1~x^3に対応するエージェント，2行目がx^4～x^6に対応するエージェントと点の距離
            % 計量の計算
            obj.G(1,1) = 1-sum(obj.r_M./r(1,:));
            obj.G(2:6,:) = 0;     % 一旦ほぼゼロに
            obj.G(:,2:6) = 0;
            %{
            obj.G(2:6,1) = 0;     % 時不変
            obj.G(1,2:6) = 0;
            obj.G(2:6,4) = 0;     % 時不変
            obj.G(4,2:6) = 0;
            %}
            obj.G(4,4) = 1-sum(obj.r_M./r(2,:));
            %{
            for alpha = [2,3,5,6]
                for beta = [2,3,5,6]
                    obj.G(alpha,beta) = -1*(alpha==beta)-sum(obj.r_M.*(r_vec(alpha,:)).*(r_vec(beta,:))./r.^3);
                    % 各点に対してr_M*x^\alpha x^beta/r^3 を計算したものが列ベクトルに入って，その行和を取ってスカラーに
                end
            end
            %}
            for alpha = [2,3]   % ブロック行列になるように変更．例えばg_{26}とかはゼロに
                for beta = [2,3]
                    obj.G(alpha,beta) = -1*(alpha==beta)-sum(obj.r_M.*(r_vec(alpha,:)).*(r_vec(beta,:))./r(1,:).^3);
                    % 各点に対してr_M*x^\alpha x^beta/r^3 を計算したものが列ベクトルに入って，その行和を取ってスカラーに
                end
            end
            for alpha = [5,6]
                for beta = [5,6]
                    obj.G(alpha,beta) = -1*(alpha==beta)-sum(obj.r_M.*(r_vec(alpha,:)).*(r_vec(beta,:))./r(2,:).^3);
                    % 各点に対してr_M*x^\alpha x^beta/r^3 を計算したものが列ベクトルに入って，その行和を取ってスカラーに
                end
            end
            obj = obj.calcSymbolicGeometry();   % TODO : 
        end

        function obj = setRingPointPotential(obj,r_M,r_size,center_points) % 半径を持ったリング状の重力源
            arguments
                obj
                r_M             % シュヴァルツシルト半径．各種定数を押し込めて使っている．1×点数
                r_size          % 質点の半径．1×点数
                center_points   % 点ポテンシャル源の位置 次元×点数
            end
            obj.r_M = r_M;
            num_point = length(r_M);    % 点の数
            r = sqrt(sum((obj.x(2:obj.dim,1)-center_points).^2,1))-r_size;
            obj.G(1,1) = 1-sum(obj.r_M./r);
            obj.G(2:obj.dim,1) = 0;     % 時不変
            obj.G(1,2:obj.dim) = 0;
            for alpha = 2:obj.dim
                for beta = 2:obj.dim
                    obj.G(alpha,beta) = -1*(alpha==beta)-sum(obj.r_M.*(obj.x(alpha,1)-center_points(alpha-1,:)).*(obj.x(beta,1)-center_points(beta-1,:))./r.^3);
                    % 各点に対してr_M*x^\alpha x^beta/r^3 を計算したものが列ベクトルに入って，その行和を取ってスカラーに
                end
            end
            obj = obj.calcSymbolicGeometry();
        end

        % クリストフェル第2種の数値解
        function Gamma2 = calcChristoffelNum(obj,x_num)
            arguments
                obj
                x_num   % 各座標の数値が入った dim*1のベクトル
            end
            Gamma2 = double(subs(obj.Gamma2,obj.x,x_num));  % シンボリックで計算してあるので，値入れるだけ
            disp(string(max(Gamma2,[],'all'))+","+string(min(Gamma2,[],'all')));
        end

        % ポテンシャルそのものの偏微分の数値解（比較用）
        function partialPhi = calcPartialPhiNum(obj,x_num)
            arguments
                obj
                x_num   % 各座標の数値が入った dim*1のベクトル
            end
            partialPhi = permute(double(subs(obj.partialG(1,1,:),obj.x,x_num)),[3,1,2]);    % g_00の偏微分をそのまま返せばよい
        end
        
        % 各点における計量行列の数値解を返す
        function metric = calcMetricNum(obj,x_num)
            arguments
                obj
                x_num   % 各座標の数値が入った dim*1のベクトル
            end
            metric = double(subs(obj.G,obj.x,x_num));   % 計量行列に値いれてそのまま渡す
        end

        % クリストフェル第2種の数値解．計量の逆行列は数値的に求める
        function Gamma2 = calcChristoffelNumWithGinv(obj,x_num)
            arguments
                obj
                x_num   % 各座標の数値が入った dim*1のベクトル
            end
            Ginv_ = inv(double(subs(obj.G,obj.x,x_num)));   % 数値的に逆行列を計算
            Gamma2_ = subs(obj.Gamma2,obj.Ginv,Ginv_);   % 逆行列のシンボリックに代入
            Gamma2 = double(subs(Gamma2_,obj.x,x_num));  % シンボリックで計算してあるので，値入れるだけ
        end

        % 偏微分以外は数値で計算する
        function Gamma2_ = calcChristoffelNumWithoutpartialG(obj,x_num)
            arguments
                obj
                x_num   % 各座標の数値が入った dim*1のベクトル
            end
            partialG_ = double(subs(obj.partialG,obj.x,x_num));
            Ginv_ = inv(double(subs(obj.G,obj.x,x_num)));   % 数値的に逆行列を計算
            Gamma1_ = zeros(obj.dim,obj.dim,obj.dim);   % クリストフェル1種の数値計算
            for i = 1:obj.dim
                for j = 1:obj.dim
                    for k = 1:obj.dim
                        Gamma1_(i,j,k) = 1/2*(partialG_(i,k,j)+partialG_(k,j,i)-partialG_(i,j,k));
                    end
                end
            end
            Gamma2_ = tensorprod(Gamma1_,Ginv_,3,1);
            %Gamma2__ = permute(tensorprod(Ginv_,Gamma1_,2,3),[2,3,1]);
            %{
            for i = 1:obj.dim
                for j = 1:obj.dim
                    for k = 1:obj.dim
                        %obj.Gamma2(i,j,k) = obj.Ginv(k,1)*obj.Gamma1(i,j,1)+Ginv(k,2)*Gamma1(i,j,2)+Ginv(k,3)*Gamma1(i,j,3)+Ginv(k,4)*Gamma1(i,j,4);
                        obj.Gamma2(i,j,k) = sum(obj.Ginv(k,:).*permute(obj.Gamma1(i,j,:),[1,3,2]));
                    end
                end
            end
            %}
        end

    end % methods

    methods (Access = protected)

        function obj = setBasicMinkowskiMetric(obj) % とりあえず計量としてミンコフスキ計量を作る
            obj.G(:,:) = 0; % 基本的に全成分0
            diagG = -1*ones(obj.dim,1);      % 対角成分は大体-1
            diagG(1:3:obj.dim) = 1;          % 3回に1回1（時間座標）
            obj.G = diag(diagG);    
        end

        function obj = addMetricPertubation(obj, G_perturbation)     % 作った摂動を計量に足しこむ
            arguments
                obj
                G_perturbation  % 計量の摂動成分
            end
            obj.G = obj.G + G_perturbation;
        end

        % シンボリックに各量を計算する
        function obj = calcSymbolicGeometry(obj)
            disp("計量行列の偏微分を行っています…")
            % 計量行列の微分
            for i = 1:obj.dim
                for j = 1:obj.dim
                    for k = 1:obj.dim
                        obj.partialG(i,j,k) = diff(obj.G(i,j),obj.x(k));
                    end
                end
            end
            simplify(obj.partialG); 
            % 第1種クリストフェル記号
            disp("第一種クリストフェル記号を計算しています…")
            for i = 1:obj.dim
                for j = 1:obj.dim
                    for k = 1:obj.dim
                        obj.Gamma1(i,j,k) = 1/2*(obj.partialG(i,k,j)+obj.partialG(k,j,i)-obj.partialG(i,j,k));
                    end
                end
            end
            simplify(obj.Gamma1); 
            % 第2種クリストフェル記号
            disp("計量の逆行列を計算しています")
            %obj.Ginv = inv(obj.G);
            G_ = sym('g_%d%d',[obj.dim,obj.dim]);   % 一発で逆行列出そうとしたら滅茶苦茶重かったので，一旦置き換え
            Ginv_ = inv(G_);
            obj.Ginv = subs(Ginv_, G_, obj.G);
            simplify(obj.Ginv); 
            disp("第二種クリストフェル記号を計算しています…")
            for i = 1:obj.dim
                for j = 1:obj.dim
                    for k = 1:obj.dim
                        %obj.Gamma2(i,j,k) = obj.Ginv(k,1)*obj.Gamma1(i,j,1)+Ginv(k,2)*Gamma1(i,j,2)+Ginv(k,3)*Gamma1(i,j,3)+Ginv(k,4)*Gamma1(i,j,4);
                        obj.Gamma2(i,j,k) = sum(obj.Ginv(k,:).*permute(obj.Gamma1(i,j,:),[1,3,2]));
                    end
                end
            end
            disp("式の単純化を行っています…")
            simplify(obj.Gamma2); 
        end % calcSymbolicGeometry

        function obj = calcSymbolicGeometryWithoutGinv(obj) % Ginvの計算はシンボリックではなく，数値で後でやる
            disp("計量行列の偏微分を行っています…")
            % 計量行列の微分
            for i = 1:obj.dim
                for j = 1:obj.dim
                    for k = 1:obj.dim
                        obj.partialG(i,j,k) = diff(obj.G(i,j),obj.x(k));
                    end
                end
            end
            simplify(obj.partialG); 
            % 第1種クリストフェル記号
            disp("第一種クリストフェル記号を計算しています…")
            for i = 1:obj.dim
                for j = 1:obj.dim
                    for k = 1:obj.dim
                        obj.Gamma1(i,j,k) = 1/2*(obj.partialG(i,k,j)+obj.partialG(k,j,i)-obj.partialG(i,j,k));
                    end
                end
            end
            simplify(obj.Gamma1); 
            % 第2種クリストフェル記号
            disp("計量の逆行列を計算しています")
            obj.Ginv = sym('ginv_%d%d',[obj.dim,obj.dim]);  % 一旦代入しないままで続ける
            disp("第二種クリストフェル記号を計算しています…")
            for i = 1:obj.dim
                for j = 1:obj.dim
                    for k = 1:obj.dim
                        %obj.Gamma2(i,j,k) = obj.Ginv(k,1)*obj.Gamma1(i,j,1)+Ginv(k,2)*Gamma1(i,j,2)+Ginv(k,3)*Gamma1(i,j,3)+Ginv(k,4)*Gamma1(i,j,4);
                        obj.Gamma2(i,j,k) = sum(obj.Ginv(k,:).*permute(obj.Gamma1(i,j,:),[1,3,2]));
                    end
                end
            end
            %disp("式の単純化を行っています…")
            %simplify(obj.Gamma2); 
        end

        function obj = calcPartialG(obj)  % 計量行列だけシンボリックで計算しておく方法
            disp("計量行列の偏微分を行っています…")
            % 計量行列の微分
            for i = 1:obj.dim
                for j = 1:obj.dim
                    for k = 1:obj.dim
                        obj.partialG(i,j,k) = diff(obj.G(i,j),obj.x(k));
                    end
                end
            end
            simplify(obj.partialG); 
        end

    end % methods
end % class


