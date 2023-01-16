
% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulation = SwarmWithWaveInteractionSimulation();                 % オブジェクトの定義
simulation.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
simulation = simulation.setParam("environment_file","setting_files/environments/free.m");   % パラメタ変更
simulation = simulation.setParam("placement_file","setting_files/init_conditions/free_Na_2.m");   % パラメタ変更
simulation = simulation.setParam("dxdt_0",[[0 0];[0 0]]);   % パラメタ変更
simulation = simulation.readSettingFiles(); % 設定ファイルの読み込み
simulation = simulation.initializeVariables();  % 初期値の計算
simulation = simulation.simulate(); % シミュレーションの実施
simulation.placePlot(399);

figure
plot(simulation.t_vec, permute(vecnorm(simulation.x(1,:,:)-simulation.x(2,:,:),2,2),[3,1,2]));
xlabel("時刻 t")
ylabel("相対距離")

% simulation = simulation.generateMovie();