classdef Simulator
    %SIMULATOR シミュレータでよく使う機能をまとめたクラス
    %       パラメータの変更機能，動画の撮影機能などを入れる予定．継承して色々なシミュレータに利用してほしい
    
    properties
        param   % パラメータの入った構造体オブジェクト
    end
    
    methods
        function obj = Simulator()
            % コンストラクタ
            obj.param = struct;   % paramオブジェクトの生成
            obj = obj.setDefaultParameters();
        end
        
        function obj = setDefaultParameters(obj)
            % パラメータとデフォルト値を設定．継承で好きにいじってほしい
            obj.param.dt = 0.01;
        end

        function obj = setParam(obj, field, value)
            % 1つのパラメターを変更．フィールド名，ないものは警告出す．外から使う
            arguments
                obj             % Simulatorオブジェクト
                field string      % パラメータ名（文字ベクトル）
                value = 0       % パラメータ初期値．省略なら0が入る
            end
            if isfield(obj.param, field)
                obj.param.field = value;
            else
                disp("WARNIG：パラメータに "+field+" は存在しません");
            end
        end

        function obj = setMultiParam(obj, list)
            % 複数パラメータを同時に変更
            arguments
                obj         % Simulatorオブジェクト
                list        % フィールド名と値が並んだ配列．["Nt", 10, "use_CBF", true] みたいに使う
            end
            if mod(length(list),2) == 0
                paramN = length(list)/2;
                for n = 1:paramN
                    obj = obj.setParam(list(2*n-1), list(2*n));
                end
            else
                disp("WARNIG : パラメータリストの長さは，2の整数倍である必要があります");
            end
        end

        function obj = setFigureProperty(obj, preset)
            % 図のプロパティを変更．MATLABのオリジナルはちょっと文字が小さいので
            % 継承先では自分オリジナルのプリセットを作っても良いと思う
            arguments
                obj
                preset {mustBeMember(preset, ["default", "large"])} = "default" % 標準でdefault設定に
            end
            if preset == "default"
                set(0,"DefaultAxesFontSize",'default');
                set(0,"DefaultLineLineWidth",'default');
                set(0,"DefaultAxesXGrid",'default');
                set(0,"DefaultAxesYGrid",'default');
            end
            if preset == "large"
                % 論文掲載できる位の文字サイズ，線太さに．グリッドも標準でつける．
                set(0,"DefaultAxesFontSize",13);    % フォントサイズ13
                set(0,"DefaultLineLineWidth",2);    % 線の太さ2
                set(0,"DefaultAxesXGrid",'on');     % X軸方向のグリッドON
                set(0,"DefaultAxesYGrid",'on');     % Y軸方向のグリッドON
            end
        end

    end
end

