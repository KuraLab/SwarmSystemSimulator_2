
% 結合振動子が同期するだけのシミュレーションを回す
% こちらのファイルを実行する
% 固有角速度と結合強度を変えながら実行し，結果をまとめて描画する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulation = SyncronizingCoupledOscillator();           % オブジェクトの定義
simulation.setFigureProperty("large");                  % 描画の基本設定を変更

%% 複数回回す設定
Nsim = 6;
omega_sets = ones(5,Nsim);  % 試すパラメタの集合
omega_sets(1,4:6) = 2;
kappa_sets = [1,2,5,1,2,5];

%% 個々のシミュレーション
t = tiledlayout(2,3,'TileSpacing','Compact');   % 描画用

for n_sim = 1:Nsim
    nexttile
    % パラメタセット
    simulation = simulation.setParam("omega_0",omega_sets(:,n_sim));
    simulation = simulation.setParam("kappa",kappa_sets(n_sim));
    % 初期値の計算
    simulation = simulation.initializeVariables();
    % シミュレーションの実施
    simulation = simulation.simulate(); 
    % 描画
    simulation = simulation.subplot();
    title("\omega_1 = "+string(omega_sets(1,n_sim))+", \kappa = "+string(kappa_sets(n_sim)))
    ylim([-2,0.1])
end

title(t,"位相差の時間変化")
xlabel(t,"時刻 s")
ylabel(t,"位相")
