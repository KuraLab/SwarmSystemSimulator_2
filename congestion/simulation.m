% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../SwarmSystemSimulator_2/"))    % パスを通す
simulator = CongestionSimulator();                         % オブジェクトの定義
simulator.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
% 杉山モデル
%simulator = simulator.setParam("control_model","sugiyama");
simulator = simulator.setParam("k",1);   % 目標速度関数の傾き
simulator = simulator.setParam("a",1);   % 加速度
simulator = simulator.setParam("c_0",0.7);   % 初速度
% 速度FBモデル
%simulator = simulator.setParam("control_model","velocity_FB");
simulator = simulator.setParam("v_r",0.8);  % 目標速度
simulator = simulator.setParam("k_v",1);  % FBゲイン
% PD制御モデル
simulator = simulator.setParam("control_model","PD");
simulator = simulator.setParam("k_P",0.5);  % Pゲイン
simulator = simulator.setParam("k_D",1);  % Dゲイン
simulator = simulator.setParam("r_r",1);  % 目標車間

% CBF制約
simulator = simulator.setParam("CBF_enable",true);  % CBF有効化
%simulator = simulator.setParam("CBF_enable",false);
simulator = simulator.setParam("T_c",0.05);  % 制御周期
simulator = simulator.setParam("gamma",8);  % ブレーキパラメタ
simulator = simulator.setParam("devide",false);  % 分割

% 摂動 : epsilon系は全てノミナル値に対する摂動の割合を表す．
simulator = simulator.setParam("epsilon_x",0);      % 初期位置摂動 -> 全モデルに適用
simulator = simulator.setParam("epsilon_v_r",0);    % 目標速度摂動 -> 杉山，速度FBモデルに適用
simulator = simulator.setParam("epsilon_v_0",0.1);  % 初期速度摂動 -> 全モデルに適用

% シミュレーション本体
simulator = simulator.initializeVariables();  % 初期値の計算
simulator = simulator.defineSystem();         % システム設定．今回はCBF用
simulator = simulator.simulate(); % シミュレーションの実施

%% 描画
%simulator = simulator.positionLinearPlot();% 位置の描画
simulator = simulator.headwayPlot();% 車間の描画
disp(min(mod(simulator.x(2,:)-simulator.x(1,:),simulator.param.L)))
disp(max(mod(simulator.x(2,:)-simulator.x(1,:),simulator.param.L)))
%simulator = simulator.generateMovie();          % 動画

% save results.mat simulator % 保存