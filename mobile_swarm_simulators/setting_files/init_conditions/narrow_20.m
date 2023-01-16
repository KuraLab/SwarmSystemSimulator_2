
% 狭所用２０台

% 台数の定義
posset_Na = 20;

% 配置の定義．
posset_x = zeros(4,5);
posset_y = zeros(4,5);

posset_x(1,:) = -4;
posset_x(2,:) = -5;
posset_x(3,:) = -6;
posset_x(4,:) = -7;
posset_y(1,:) = [2.4 1.2 0 -1.2 -2.4];
posset_y(2,:) = [3.1 1.9 0.7 -0.7 -1.9];
posset_y(3,:) = [2.4 1.2 0 -1.2 -2.4];
posset_y(4,:) = [3.1 1.9 0.7 -0.7 -1.9];

posset_x = reshape(posset_x.',[posset_Na,1]);
posset_y = reshape(posset_y.',[posset_Na,1]);