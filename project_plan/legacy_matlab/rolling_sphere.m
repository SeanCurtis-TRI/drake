%% Accuracy values (shared across all cases)
accuracy = [1e-1, 1e-2, 1e-3, 1e-4];

% All runs are a single sphere:
% radius = 0.1 m 
% mass 0.1 Kg 
% mu 0.3
% E = 1e9 Pa
% d = 10 s/m

%% Case 1: z = 0.3 m, vx = 1.0 m/s
x_f_cenic_main_hydro_z03 = [0.15985220574763118, 0.13647117167944542, 1.4097852371768318, 2.1645406586063847];
x_f_barrier_0_z03 = [0.15768079733600596, 0.1315490600139106, 1.2188586771211853, 1.8110278312675192];
x_f_barrier_1e_4_z03 = [1.144670135987892, 1.2042184185442162, 1.375513518468263, 1.9208873628173224];

figure;
semilogx(accuracy, x_f_cenic_main_hydro_z03, '-o', 'LineWidth', 1); hold on;
semilogx(accuracy, x_f_barrier_0_z03, '-s', 'LineWidth', 1);
semilogx(accuracy, x_f_barrier_1e_4_z03, '-^', 'LineWidth', 1);
hold off;

ylabel('x_{final} [m]');
xlabel('accuracy');
title('z = 0.3 m, v_x = 1.0 m/s');
legend('volumetric hydro (cenic main)', 'barrier \delta = 0', 'barrier \delta = 1e-4', ...
       'Location', 'southwest');
grid on;

% Export figure
exportgraphics(gcf, 'case_z0p3_vx1_accuracy_vs_xfinal.png', 'Resolution', 600);

%% Case 2: z = 0.1 + 2\delta, v_x = 1.0 m/s, \omega_y = 10 rad/s
x_f_cenic_main_hydro_spin = [0.14716159725871472, 0.8448730514136108, 1.965651334668332, 3.137960778047583];
x_f_barrier_0_spin = [0.304862899143438, 1.0083654458342581, 1.9355082972233757, 2.5700823000192767];
x_f_barrier_1e_4_spin = [0.9148595059140874, 0.941388047367457, 1.7594608011449306, 2.7586004176183323];

figure;
semilogx(accuracy, x_f_cenic_main_hydro_spin, '-o', 'LineWidth', 1); hold on;
semilogx(accuracy, x_f_barrier_0_spin, '-s', 'LineWidth', 1);
semilogx(accuracy, x_f_barrier_1e_4_spin, '-^', 'LineWidth', 1);
hold off;

ylabel('x_{final} [m]');
xlabel('accuracy');
title('z = 0.1 + 2\delta, v_x = 1.0 m/s, \omega_y = 10 rad/s');
legend('volumetric hydro (cenic main)', 'barrier \delta = 0', 'barrier \delta = 1e-4', ...
       'Location', 'southwest');
grid on;

% Export figure
exportgraphics(gcf, 'case_spin_accuracy_vs_xfinal.png', 'Resolution', 600);

%% Case 3: z = 0.1 + 2\delta, v_x = 0.5 m/s, \omega_y = 5 rad/s
x_f_cenic_main_hydro_spin_05 = [0.2043681609848593, 0.23450455179575863, 0.6247588231553453, 1.1208164706491732];
x_f_barrier_0_spin_05 = [0.22671902292214557, 0.3153412364479043, 0.6456185550448655, 1.073645121001138];
x_f_barrier_1e_4_spin_05 = [0.2100483152674973, 0.3950219819692653, 0.6061547398325728, 1.1437659892270375];

figure;
semilogx(accuracy, x_f_cenic_main_hydro_spin_05, '-o', 'LineWidth', 1); hold on;
semilogx(accuracy, x_f_barrier_0_spin_05, '-s', 'LineWidth', 1);
semilogx(accuracy, x_f_barrier_1e_4_spin_05, '-^', 'LineWidth', 1);
hold off;

ylabel('x_{final} [m]');
xlabel('accuracy');
title('z = 0.1 + 2\delta, v_x = 0.5 m/s, \omega_y = 5 rad/s');
legend('volumetric hydro (cenic main)', 'barrier \delta = 0', 'barrier \delta = 1e-4', ...
       'Location', 'northeast');
grid on;

% Export figure
exportgraphics(gcf, 'case_spin_05_accuracy_vs_xfinal.png', 'Resolution', 600);

%% Case 4: z = 0.1 + 0.1*\delta, v_x = 0.5 m/s, \omega_y = 5 rad/s
x_f_cenic_main_hydro_spin_05_delta_01 = [0.2011997585083154, 0.272256165653805, 0.5774526722608849, 1.1313579152778521];
x_f_barrier_0_spin_05_delta_01 = [0.2160449266105036, 0.3493319716826102, 0.6545068027726325, 1.0651980664483953];
x_f_barrier_1e_4_spin_05_delta_01 = [0.20064490611057492, 0.34311287896711823, 0.6250345911467753, 1.1377366055188802];

figure;
semilogx(accuracy, x_f_cenic_main_hydro_spin_05_delta_01, '-o', 'LineWidth', 1); hold on;
semilogx(accuracy, x_f_barrier_0_spin_05_delta_01, '-s', 'LineWidth', 1);
semilogx(accuracy, x_f_barrier_1e_4_spin_05_delta_01, '-^', 'LineWidth', 1);
hold off;

ylabel('x_{final} [m]');
xlabel('accuracy');
title('z = 0.1 + 0.1 * \delta, v_x = 0.5 m/s, \omega_y = 5 rad/s');
legend('volumetric hydro (cenic main)', 'barrier \delta = 0', 'barrier \delta = 1e-4', ...
       'Location', 'northeast');
grid on;

% Export figure
exportgraphics(gcf, 'case_spin_05_delta_01_accuracy_vs_xfinal.png', 'Resolution', 600);