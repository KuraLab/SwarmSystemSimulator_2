% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../SwarmSystemSimulator_2/"))    % パスを通す
simulator = CongestionSimulator();                         % オブジェクトの定義
simulator.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
simulator = simulator.setParam("k",0);   % パラメタ変更
simulator = simulator.setParam("c_0",1);   % パラメタ変更
simulator = simulator.initializeVariables();  % 初期値の計算
simulator = simulator.simulate(); % シミュレーションの実施
simulator = simulator.positionLinearPlot();% 描画

% save results.mat simulator % 保存