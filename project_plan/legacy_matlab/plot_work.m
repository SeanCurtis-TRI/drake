
accuracy      = [1e-1, 1e-2, 1e-3, 1e-4];

% E = 1e7, margin = 1e-3, delta = 1e-4, beta = 1

% Data from 4 piles of 5 spheres.
%success       = [399, 441, 744, 2913];
%error_control = [0, 15, 339, 1392];
%feasibility   = [272, 199, 52, 22]

success       = [900, 984, 2595, 7635];
error_control = [0, 102, 1281, 4539];
feasibility   = [607, 296, 82, 13];

colors = lines(3);


figure("position", [0, 0, 3840, 2160]);
semilogx(accuracy, success, '-o', 'Color', colors(1, :), 'DisplayName', 'success');
hold on;
semilogx(accuracy, error_control, '-o', 'Color', colors(2, :), 'DisplayName', 'erro_control');
hold on;
semilogx(accuracy, feasibility, '-o', 'Color', colors(3, :), 'DisplayName', 'feasibility');
hold on;
xlabel('Accuracy'); ylabel('# Convex solves');
title('# Convex solves vs accuracy');
legend('show'); legend('location', 'best', 'box', 'off');
grid on; box on;
