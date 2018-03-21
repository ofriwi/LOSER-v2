torques = 1:250;
speeds = zeros(length(torques), length(motors));
for i = 1 : length(torques)
    tor = torques(i);
    rats = tor./motors(:, 3);
    rats = 1 - rats;
    speeds(i, :) = max(motors(:, 2) .* rats(:), 0);
end

for j = 1: length(motors)
    plot(torques, speeds(:,j));
    [num2str(j), '-', num2str(motors(j,1))]
    hold on
end