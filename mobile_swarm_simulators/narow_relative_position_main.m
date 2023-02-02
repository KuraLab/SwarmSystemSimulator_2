

% こちらのファイルを実行する
% 一発実施する

%% 最初に1回だけ回すもの．おまじない
addpath(genpath("../../../SwarmSystemSimulator_2/"))    % パスを通す
simulation = SwarmWithWaveInteractionSimulation();                 % オブジェクトの定義
simulation.setFigureProperty("large");                  % 描画の基本設定を変更

%% 誘導場の生成


%% シミュレーションの実施 : 単発
simulation = simulation.setParam("environment_file","setting_files/environments/narrow_space.m");   % パラメタ変更
simulation = simulation.setParam("placement_file","setting_files/init_conditions/narrow_20.m");   % パラメタ変更
simulation.cos = simulation.cos.setParam("kappa",100);
simulation.cos = simulation.cos.setParam("do_estimate",true);
simulation.cos = simulation.cos.setParam("time_histry",256);
simulation.cos = simulation.cos.setParam("power_threshold",10^-7);
%simulation.cbf = simulation.cbf.disable();
simulation.cbf.gamma = 5;   % CBF : ナイーブ係数
simulation.cbf.rs = 0.8;    % CBF : 安全距離
simulation = simulation.setParam("kp",8);   % Swarm : 勾配追従力ゲイン
simulation = simulation.setParam("kf",0);  % Swarm : 群形成力ゲイン
simulation = simulation.setParam("kd",10);   % Swarm : 粘性ゲイン
simulation = simulation.setParam("Nt",3000);
simulation = simulation.setParam("is_debug_view",false);
simulation = simulation.setParam("initial_pos_variance", 0.0);
simulation = simulation.setParam("attract_force_type", "linear_fbx");
simulation = simulation.setParam("do_kp_adjust",true);  % kp調整を実施？
simulation = simulation.setParam("kp_adjust_out",-0.3);
%simulation = simulation.setParam("kp_adjust_in",-0.3);
simulation = simulation.setParam("kp_adjust_in",1.2);
simulation = simulation.setParam("adjust_stepwith",80);
%simulation = simulation.setParam("dxdt_0",[[0 0];[0 0]]);   % パラメタ変更
simulation = simulation.readSettingFiles(); % 設定ファイルの読み込み
simulation = simulation.initializeVariables();  % 初期値の計算
simulation = simulation.defineSystem();  % システム設定（誘導場の生成）
simulation = simulation.simulate(); % シミュレーションの実施
%% 描画とか
figure
simulation.edgeDeadlockPlot(183,2);
simulation.placePlot(1);
% simulation.cos = simulation.cos.plot();
% simulation = simulation.generateMovieEstimate();
simulation = simulation.generateMovieEstimate();
simulation = simulation.setParam("is_debug_view",true);
simulation = simulation.calcControlInput(750);
simulation.cos.relativePositionEstimate(750,[8,9,10]);  % 推定デバッグ表示
simulation.cos.peakAndFreqPlot([8,9,10]);   % エージェント毎ピーク履歴
simulation.cos.peakAndFreqPlot2([1,5:20]);   % モード毎ピーク履歴
% simulation.cos.spectrumPlot(750,8);   % 特定時刻スペクトラムプロット
% simulation.cos.generateSpectrumMovie();
% simulation.cos.deadlockPlot([1,5:20]);
% simulation.cos.variancePlot([1,5:20]);
% simulation.kpAdjustPlot([1,5:20]);
%{ 
figure
subplot(2,2,1)
simulation.trajectryJudgePlot([2:400]);
subplot(2,2,2)
simulation.trajectryJudgePlot([401:800]);
subplot(2,2,3)
simulation.trajectryJudgePlot([801:1200]);
subplot(2,2,4)
simulation.trajectryJudgePlot([1201:1600]);
figure
subplot(1,2,1)
simulation.trajectryJudgePlot([1601:2000]);
subplot(1,2,2)
simulation.trajectryJudgePlot([2001:3000]);
%}
%% シミュレーションの実施 : 回す
kp_list = [0.5, 1, 2, 4, 8, 10];
kf_list = [0.1, 1, 10, 20, 40, 60];
%kp_list = [1 5];
%kf_list = [10];
number_of_sets = length(kp_list)*length(kf_list);
sim_per_sets = 20;
results = table('size',[number_of_sets,3],'VariableTypes',["double","double","double"]);
results.Properties.VariableNames = ["kp","kf","number_of_deadlock"];
sets_count = 0;
for kp = kp_list
    for kf = kf_list
        sets_count = sets_count+1;
        deadlock_count = 0;
        for n = 1:sim_per_sets  % 1セット
            clc
            disp(string(sets_count)+"/"+string(number_of_sets)+"セット，"+string(n)+"/"+string(sim_per_sets)+"試行");

            simulation = simulation.setParam("environment_file","setting_files/environments/narrow_space.m");   % パラメタ変更
            simulation = simulation.setParam("placement_file","setting_files/init_conditions/narrow_20.m");   % パラメタ変更
            simulation.cos = simulation.cos.setParam("kappa",30);
            %simulation.cbf = simulation.cbf.disable();
            simulation.cbf.gamma = 5;   % CBF : ナイーブ係数
            simulation.cbf.rs = 0.8;    % CBF : 安全距離
            simulation = simulation.setParam("kp",kp);   % Swarm : 勾配追従力ゲイン
            simulation = simulation.setParam("kf",kf);  % Swarm : 群形成力ゲイン
            simulation = simulation.setParam("kd",20);   % Swarm : 粘性ゲイン
            simulation = simulation.setParam("Nt",400);
            simulation = simulation.setParam("is_debug_view",false);
            simulation = simulation.setParam("initial_pos_variance", 0.05);
            simulation = simulation.readSettingFiles(); % 設定ファイルの読み込み
            simulation = simulation.initializeVariables();  % 初期値の計算
            simulation = simulation.defineSystem();  % システム設定（誘導場の生成）
            simulation = simulation.simulate(); % シミュレーションの実施
            
            if simulation.is_connected == false
                deadlock_count = deadlock_count+1;
            end
        end
        results(sets_count,:) = {kp,kf,deadlock_count};
    end
end
