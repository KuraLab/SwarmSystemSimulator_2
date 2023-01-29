

% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulation = SwarmWithWaveInteractionSimulation();                 % オブジェクトの定義
simulation.setFigureProperty("large");                  % 描画の基本設定を変更

%% 誘導場の生成


%% シミュレーションの実施
simulation = simulation.setParam("environment_file","setting_files/environments/narrow_space.m");   % パラメタ変更
simulation = simulation.setParam("placement_file","setting_files/init_conditions/narrow_20_grid.m");   % パラメタ変更
simulation.cos = simulation.cos.setParam("kappa",30);
simulation.cbf = simulation.cbf.disable();
simulation = simulation.setParam("kp",0);
simulation = simulation.setParam("kd",0);
%simulation = simulation.setParam("dxdt_0",[[0 0];[0 0]]);   % パラメタ変更
simulation = simulation.readSettingFiles(); % 設定ファイルの読み込み
simulation = simulation.initializeVariables();  % 初期値の計算
simulation = simulation.defineSystem();  % システム設定（誘導場の生成）
simulation = simulation.simulate(); % シミュレーションの実施
figure
simulation.placePlot(399);
% simulation.cos = simulation.cos.plot();
% simulation = simulation.generateMovie();

%% TODO
% 群形成力か粘性入れないとダメかも
% とりあえず群形成力入れたら先に振動相互作用
% 昔のは粘性入っている -> CBFのシステムからして違くない？