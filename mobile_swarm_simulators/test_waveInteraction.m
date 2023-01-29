% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulation = WaveInteractionSimulator();                 % オブジェクトの定義
simulation.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
%simulation = simulation.setParam("K",2);   % パラメタ変更
simulation = simulation.setParam("phi_0",[1; 0]);   % パラメタ変更
simulation = simulation.setParam("omega_0",[1; 0]);
simulation = simulation.setParam("interaction_type","diffusion");
simulation = simulation.initializeVariables();  % 初期値の計算
G = graph([0 1; 1 0]);
simulation = simulation.setGraph(G);
simulation = simulation.simulate(); % シミュレーションの実施
%simulation.plot(399);
simulation.phaseGapPlot(399);