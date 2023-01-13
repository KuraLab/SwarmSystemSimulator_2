% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulation = MobileRobots2dSimulator();                 % オブジェクトの定義
simulation.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
simulation = simulation.setParam("K",2);   % パラメタ変更
simulation = simulation.readSettingFiles(); % 設定ファイルの読み込み
simulation = simulation.initializeVariables();  % 初期値の計算
simulation = simulation.simulate(); % シミュレーションの実施
simulation.placePlot(399);

%rv = simulation.calcVectorToWalls(200);
%rv_ = permute(rv(1,:,:),[3,2,1]);
%quiver(simulation.x(1,1,200)*ones(12,1),simulation.x(1,2,200)*ones(12,1),rv_(:,1),rv_(:,2));
%simulation = simulation.plot();% 描画

% save results.mat simulation % 保存
% simulation = simulation.generateMovie();