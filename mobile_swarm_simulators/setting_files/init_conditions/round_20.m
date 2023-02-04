
% 狭所用２０台

% 台数の定義
posset_Na = 20;

% 配置の定義．
posset_x = zeros(4,5);
posset_y = zeros(4,5);

posset_x(1,:) = -6;
posset_x(2,:) = -7;
posset_x(3,:) = -8;
posset_x(4,:) = -9;
posset_y(1,:) = [8.4 7.2 6.0 4.8 3.6];
posset_y(2,:) = [9.1 7.9 6.7 5.5 4.3];
posset_y(3,:) = [8.4 7.2 6.0 4.8 3.6];
posset_y(4,:) = [9.1 7.9 6.7 5.5 4.3];

posset_x = reshape(posset_x.',[posset_Na,1]);
posset_y = reshape(posset_y.',[posset_Na,1]);