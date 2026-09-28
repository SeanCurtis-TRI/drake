accuracy = [1e-1, 1e-2, 1e-3, 1e-4, 1e-6, 1e-8, 1e-10];

%% Case 3: z = 0.1 + 2\delta, v_x = 0.5 m/s, \omega_y = 5 rad/s
x_f_cenic_main_hydro = [0.32712258386913945, 0.36006005273403663, 0.6689915140782494, 1.21695014833409, 3.7184415239495547, 4.5604435587530565, 4.6639132791951];
x_f_barrier_0 = [0.29112367425716656, 0.38550845175728327, 0.7137581903184426, 1.1852613159002956, 3.5154448837440593, 4.232194613966953, 4.3131997408557865];
x_f_barrier_1e_4 = [0.31188201121064313, 0.3971087669533448, 0.7780574114319461, 1.3546947851826812, 3.170818795528213, 3.626739052570931, 3.6995348313297334];

figure;
semilogx(accuracy, x_f_cenic_main_hydro, '-o', 'LineWidth', 1); hold on;
semilogx(accuracy, x_f_barrier_0, '-s', 'LineWidth', 1);
semilogx(accuracy, x_f_barrier_1e_4, '-^', 'LineWidth', 1);
hold off;

ylabel('x_{final} [m]');
xlabel('accuracy');
title('z = 0.1 + 2\delta, v_x = 0.5 m/s, \omega_y = 5 rad/s');
legend('volumetric hydro (cenic main)', 'barrier \delta = 0', 'barrier \delta = 1e-4', ...
       'Location', 'northeast');
grid on;

% Export figure
exportgraphics(gcf, 'rolling_sphere.png', 'Resolution', 600);
