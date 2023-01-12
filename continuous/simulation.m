
% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulator = continuousSimulator();                         % オブジェクトの定義
simulator.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
%x_vec = 0:simulator.param.dx:simulator.param.dx*(simulator.param.Nx-1);
%phi_0 = normpdf(x_vec, x_vec(round(simulator.param.Nx/2)),1);
phi_0 = zeros(simulator.param.Nx,1);
%phi_0(round(simulator.param.Nx/2)) = 1;
simulator = simulator.setParam("phi_0",phi_0);   % パラメタ変更
simulator = simulator.setParam("dx",1);   % パラメタ変更
simulator = simulator.setParam("kp",0.02);   % 群形成力
simulator = simulator.setParam("k",0.01);   % 勾配追従力
simulator = simulator.setParam("eta",0);   % 粘性係数
simulator = simulator.setParam("kappa",2);   % 結合強度
simulator = simulator.setParam("Nt",3000);   % シミュレーションカウント
simulator = simulator.setParam("Omega",0.1);   % 固有角速度

simulator = simulator.initializeVariables();  % 初期値の計算
simulator = simulator.defineSystem();
simulator = simulator.simulate(); % シミュレーションの実施
simulator = simulator.plot();% 描画

% save results.mat simulation % 保存
%% 動画
figure
simulator.moviePlot(simulator.param.Nt);
%simulator.generateMovie("movie.mp4",5);
%simulator.generateMovie2();