
% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulator = continuousSimulator();                         % オブジェクトの定義
simulator.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
phi_0 = zeros(simulator.param.Nx,1);
phi_0(round(simulator.param.Nx/2)) = 1;
simulator = simulator.setParam("phi_0",phi_0);   % パラメタ変更
simulator = simulator.setParam("dx",1);   % パラメタ変更
simulator = simulator.initializeVariables();  % 初期値の計算
simulator = simulator.defineSystem();
simulator = simulator.simulate(); % シミュレーションの実施
simulator = simulator.plot();% 描画

% save results.mat simulation % 保存
%% 動画
%simulator.generateMovie();
%simulator.generateMovie2();