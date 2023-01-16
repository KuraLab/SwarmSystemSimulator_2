% 時空間的ポテンシャルを計量として持つような時空間
classdef symboricTimeSpacePotentialMetric < symbolicMetricSpace
    properties (Access = protected)
    end

    methods (Access = public)
        function obj = symboricTimeSpacePotentialMetric(dim)    % コンストラクタ
            arguments
                dim % 時空間の次元．３の倍数であってほしい
            end
            if mod(dim,3) ~= 0    % 次元が3の倍数でない
                error("次数 dim は3の倍数としてください")
            end
            obj@symbolicMetricSpace(dim);   % 親クラスのコンストラクタ呼び出し
        end

        function obj = setPotentialMetric(obj,r_M_interaction,r_size_interaction,use_minkowski)   % 時空間を用いたポテンシャルによる相互作用を記述
            arguments
                obj
                r_M_interaction % 相互作用のシュバルツシルト半径
                r_size_interaction % エージェント半径
                use_minkowski = true % ミンコフスキ計量を使うか？
            end
            obj = obj.setBasicMinkowskiMetric();    % 標準ミンコフスキ計量をまず作る
            G_ = sym('g_%d%d',[obj.dim,obj.dim]);   % ポテンシャルによる標準的ミンコフスキ計量からのずれ
            G_(:,:) = 0;                            % 全成分0で初期化
            N = floor(obj.dim/3);                   % エージェント数
            r_vec = sym('r_vec%d%d',[obj.dim,N]);   % 他エージェントとの相対置を返すベクトルを並べたもの
            r_vec = repmat(obj.x,1,N) - obj.x(repmat(reshape(1:obj.dim,3,N),N,1)); % 差を生成 dim行n列にちゃんとなるはず
            for n = 1:N     % エージェント数で回す, 行側
                if use_minkowski
                    r = sqrt(abs(r_vec(3*n-2,:).^2-r_vec(3*n-1,:).^2-r_vec(3*n,:).^2));   % ミンコフスキノルム
                else
                    r = sqrt(abs(r_vec(3*n-2,:).^2+r_vec(3*n-1,:).^2+r_vec(3*n,:).^2));   % ユークリッドノルム
                end
                r_nonzero = r;
                r_nonzero(r==0) = 1;    % ゼロ割りの防止
                phi = -r_M_interaction./(r_nonzero-r_size_interaction);   % 自分から見える個々の相手とのポテンシャルの計算
                phi(n) = 0; % 自分とこの項はゼロ
                for m = 1:N     % 相手側，列側
                    if (m==n)   % 自分とのやつ
                        for i = 3*n-2:3*n
                            for j = 3*n-2:3*n
                                G_(i,j) = sum(phi.*r_vec(i,:).*r_vec(j,:)./(r_nonzero.^2)); % phi*x^i*x^j/r^2
                                % 全ての相手とのポテンシャルが関係する
                                %simplify(G_(i,j));   % ちょっと簡単化
                            end
                        end
                        if use_minkowski
                            %G_(3*n-1:3*n,3*n-1:3*n) = -G_(3*n-1:3*n,3*n-1:3*n);   % 空間座標なら反転せよ
                            G_(3*n-2,3*n-1:3*n) = -G_(3*n-2,3*n-1:3*n); % 符号反転を受けるのは，g_{\alpha 0}のみ
                            G_(3*n-1:3*n,3*n-2) = -G_(3*n-1:3*n,3*n-2);
                        end
                    else    % m neq n
                        for i = 3*n-2:3*n
                            for j = 3*m-2:3*m
                                G_(i,j) = phi(1,m)*r_vec(i,m)*r_vec(j,n)/(r_nonzero(1,m)^2); % phi*x^i*x^j/r^2
                                % m番目とのポテンシャルのみが関係する
                                %simplify(G_(i,j));   % ちょっと簡単化
                            end
                        end
                        if use_minkowski
                            %G_(3*n-1:3*n,3*m-1:3*m) = -G_(3*n-1:3*n,3*m-1:3*m);   % 空間座標なら反転せよ
                            G_(3*n-2,3*m-1:3*m) = -G_(3*n-2,3*m-1:3*m); % 符号反転を受けるのは，g_{\alpha 0}のみ
                            G_(3*n-1:3*n,3*m-2) = -G_(3*n-1:3*n,3*m-2);
                        end
                    end
                end %for m
            end % for n

            obj = obj.addMetricPertubation(G_);     % 作った摂動を足しこむ
            %simplify(obj.G);   % ちょっと簡単化
            %obj = obj.calcSymbolicGeometryWithoutGinv();    % 逆行列はやばいので後で代入
            obj = obj.calcPartialG();   % 偏微分だけシンボリックで事前計算
        end
    end

    methods (Access = protected)
    end
end % classdef