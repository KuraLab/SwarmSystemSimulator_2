
%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulation = simpleTimeSpace();           % オブジェクトの定義
simulation.setFigureProperty("large");                  % 描画の基本設定を変更

%% シミュレーションの実施
simulation = simulation.setParam("A",2);   % パラメタ変更
simulation = simulation.setParam("B",1);
%simulation = simulation.setParam("dqdtau_0",[100, 5]);
simulation = simulation.setParam("dqdtau_0",[1, 0.5]);
simulation = simulation.setParam("Ntau",300);
simulation = simulation.setParam("c",1);
simulation = simulation.setParam("f",0);
%simulation = simulation.setParam("metric","const_input");
simulation = simulation.setParam("metric","deSitter");
simulation = simulation.initializeVariables();  % 初期値の計算
simulation = simulation.defineSystem();         % システムの設定
simulation = simulation.simulate(); % シミュレーションの実施
simulation = simulation.plot();% 描画

%% 繰り返し実行
%f_list = [0,1,2];
%A_list = [-1,0,0.5,2,8];
%B_list = [-1,0,1,2,8];
v0_list = [1.0 2 4];
figure
for nsim = 1:length(v0_list)
    simulation = simulation.setParam("A",2);   % パラメタ変更
    simulation = simulation.setParam("B",1);
    %simulation = simulation.setParam("dqdtau_0",[100, 5]);
    simulation = simulation.setParam("dqdtau_0",[1, 0.5]);
    simulation = simulation.setParam("Ntau",300);
    simulation = simulation.setParam("c",1);
    simulation = simulation.setParam("f",0);
    simulation = simulation.setParam("metric","deSitter");
    %simulation = simulation.setParam("C",C_list(1,nsim));
    simulation = simulation.setParam("A",2);
    simulation = simulation.setParam("dqdtau_0",[1, v0_list(1,nsim)]);
    simulation = simulation.initializeVariables();  % 初期値の計算
    simulation = simulation.defineSystem();         % システムの設定
    simulation = simulation.simulate(); % シミュレーションの実施
    simulation = simulation.subplot();% 描画
    hold on
end
title("状態の世界時間変化 (c=1,f=0)")
ylabel("x^1")
xlabel("時刻 t [s]")
%legend("f="+string(f_list))
%legend("A="+string(A_list))
%legend("B="+string(B_list))
legend("v_0="+string(v0_list))

%% 繰り返し実行
%f_list = [0,1,2];
%A_list = [-1,0,0.5,2,8];
%B_list = [-1,0,1,2,8];
C_list = [-1,0,0.5,2,8];
figure
for nsim = 1:length(B_list)
    %simulation = simulation.setParam("f",f_list(1,nsim));   % パラメタ変更
    %simulation = simulation.setParam("A",A_list(1,nsim));
    %simulation = simulation.setParam("B",B_list(1,nsim));
    simulation = simulation.setParam("C",C_list(1,nsim));
    simulation = simulation.setParam("A",2);
    simulation = simulation.setParam("dqdtau_0",[100, 5]);
    simulation = simulation.initializeVariables();  % 初期値の計算
    simulation = simulation.defineSystem();         % システムの設定
    simulation = simulation.simulate(); % シミュレーションの実施
    simulation = simulation.subplot();% 描画
    hold on
end
title("状態の世界時間変化 (A=2,B=1,c=100,f=1)")
ylabel("x^1")
xlabel("時刻 t [s]")
%legend("f="+string(f_list))
%legend("A="+string(A_list))
%legend("B="+string(B_list))
legend("C="+string(C_list))

%% 繰り返し実行_固有時間表示
%f_list = [0,1,2];
%A_list = [-1,0,0.5,2,8];
%B_list = [-1,0,1,2,8];
C_list = [-1,0,0.5,2,8];
f1 = figure;
f2 = figure;
for nsim = 1:length(B_list)
    %simulation = simulation.setParam("f",f_list(1,nsim));   % パラメタ変更
    %simulation = simulation.setParam("A",A_list(1,nsim));
    %simulation = simulation.setParam("B",B_list(1,nsim));
    simulation = simulation.setParam("C",C_list(1,nsim));
    simulation = simulation.setParam("A",2);
    simulation = simulation.setParam("dqdtau_0",[100, 5]);
    simulation = simulation.initializeVariables();  % 初期値の計算
    simulation = simulation.defineSystem();         % システムの設定
    simulation = simulation.simulate(); % シミュレーションの実施
    figure(f1)
    simulation = simulation.subplot_eigentime(1);% 描画
    hold on
    figure(f2)
    simulation = simulation.subplot_eigentime(2);% 描画
    hold on
end
figure(f1)
title("状態の固有時間変化 (A=2,B=1,c=100,f=1)")
ylabel("x^0")
xlabel("固有時間 \tau [s]")
%legend("f="+string(f_list))
%legend("A="+string(A_list))
%legend("B="+string(B_list))
legend("C="+string(C_list))

figure(f2)
title("状態の固有時間変化 (A=2,B=1,c=100,f=1)")
ylabel("x^1")
xlabel("固有時間 \tau [s]")
%legend("f="+string(f_list))
%legend("A="+string(A_list))
%legend("B="+string(B_list))
legend("C="+string(C_list))

%% 矢印の描画

%f_list = [0,1,2];
%A_list = [-1,-0.5,0,0.5,2,4,8];
%B_list = [-1,-0.5,0,0.5,1,2,4,8];
%C_list = [-1,-0.5,0,0.4,1,2,4,8];
A_list = [0.5, 1, 2, 4, 8];
C_list = [8, 4, 2, 1, 0.5];
for nsim = 1:length(C_list)
    %simulation = simulation.setParam("f",f_list(1,nsim));   % パラメタ変更
    simulation = simulation.setParam("A",A_list(1,nsim));
    %simulation = simulation.setParam("B",B_list(1,nsim));
    simulation = simulation.setParam("C",C_list(1,nsim));
    %simulation = simulation.setParam("A",2);
    %simulation = simulation.setParam("C",1);
    simulation = simulation.setParam("B",1.9);
    simulation = simulation.setParam("dqdtau_0",[100, 5]);
    simulation = simulation.initializeVariables();  % 初期値の計算
    simulation = simulation.defineSystem();         % システムの設定
    %simulation = simulation.simulate(); % シミュレーションの実施
    figure
    simulation.showEigen(-100:20:100,-100:40:100);
    xlabel("t")
    ylabel("x^1")
    title("A="+string(simulation.param.A)+", B="+string(simulation.param.B)+", C="+string(simulation.param.C))
    saveas(gcf,string(nsim)+"_A="+string(simulation.param.A)+", B="+string(simulation.param.B)+", C="+string(simulation.param.C)+".png")
end
%title("計量行列の固有ベクトル (A=2,B=1,c=100,f=1)")
%ylabel("x^1")
%xlabel("時刻 t [s]")
%legend("f="+string(f_list))
%legend("A="+string(A_list))
%legend("B="+string(B_list))
%legend("C="+string(C_list))