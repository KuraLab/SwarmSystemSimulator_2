
% 狭所用42台 : 細目じゃないと怒られるので注意

% 台数の定義
posset_Na = 48;

% 配置の定義．
posset_x = zeros(6,8);
posset_y = zeros(6,8);

posset_x(1,:) = -2.1;
posset_x(2,:) = -3.1;
posset_x(3,:) = -4.1;
posset_x(4,:) = -5.1;
posset_x(5,:) = -6.1;
posset_x(6,:) = -7.1;
posset_y(1,:) = -3.15:0.9:3.15;
posset_y(2,:) = -3.15:0.9:3.15;
posset_y(3,:) = -3.15:0.9:3.15;
posset_y(4,:) = -3.15:0.9:3.15;
posset_y(5,:) = -3.15:0.9:3.15;
posset_y(6,:) = -3.15:0.9:3.15;
posset_y(:,:) = posset_y(:,:) + 0.1;
posset_x = reshape(posset_x.',[posset_Na,1]);
posset_y = reshape(posset_y.',[posset_Na,1]);