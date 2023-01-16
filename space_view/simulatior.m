% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulation = spaceViewSimulator();                         % オブジェクトの定義
simulation.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
%simulation = simulation.setParam("K",2);   % パラメタ変更
simulation = simulation.initializeVariables();  % 初期値の計算
simulation = simulation.simulate(); % シミュレーションの実施
simulation = simulation.plot();% 描画

% save results.mat simulation % 保存
%% 動画
simulation.generateMovie();
simulation.generateMovie2();