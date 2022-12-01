
%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulation = simpleTimeSpace();           % オブジェクトの定義
simulation.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
simulation = simulation.setParam("f",1);   % パラメタ変更
simulation = simulation.setParam("dqdtau_0",[100, 5]);
simulation = simulation.initializeVariables();  % 初期値の計算
simulation = simulation.defineSystem();         % システムの設定
simulation = simulation.simulate(); % シミュレーションの実施
simulation = simulation.subplot();% 描画

%% 繰り返し実行
f_list = [0,1,2];
figure
for nsim = 1:length(f_list)
    simulation = simulation.setParam("f",f_list(1,nsim));   % パラメタ変更
    simulation = simulation.setParam("dqdtau_0",[100, 5]);
    simulation = simulation.initializeVariables();  % 初期値の計算
    simulation = simulation.defineSystem();         % システムの設定
    simulation = simulation.simulate(); % シミュレーションの実施
    simulation = simulation.subplot();% 描画
    hold on
end
title("状態の世界時間変化")
ylabel("x^1")
xlabel("時刻 t [s]")
legend("f="+string(f_list))