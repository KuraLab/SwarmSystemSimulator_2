
% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulation = AttractantFieldGenerator();                 % オブジェクトの定義
simulation.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
%simulation = simulation.setParam("dxdt_0",[[0 0];[0 0]]);   % パラメタ変更
simulation = simulation.readSettingFiles(); % 設定ファイルの読み込み
simulation = simulation.initializeVariables();  % 初期値の計算
simulation = simulation.defineSystem();         % システムの初期化：ここでやっているのはactive_fieldの計算
simulation = simulation.simulate(); % シミュレーションの実施
simulation.plot(399);


% simulation = simulation.generateMovie();