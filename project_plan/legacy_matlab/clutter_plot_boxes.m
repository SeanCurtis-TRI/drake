% === Parameters ===
E_list = [1e7, 1e8, 1e9, 1e10];
delta_list = [5e-4, 7.5e-4, 1e-3];
datadir = 'clutter_data_boxes';

% Struct to hold results
results = struct();

% --- global defaults for figures ---
set (0, "defaultaxesfontname", "Helvetica");
set (0, "defaultaxesfontsize", 20);     % tick labels
set (0, "defaulttextfontsize", 26);     % titles, axis labels
set (0, "defaultlinelinewidth", 1.5);     % plot line thickness
set (0, "defaultaxeslinewidth", 1.5);   %s axis box thickness

% === Loop over files ===
for Ei = 1:numel(E_list)
    for di = 1:numel(delta_list)
        E = E_list(Ei);
        d = delta_list(di);

        % Construct filename
        fname = sprintf('%s/E_%g_d_%.0e.txt', datadir, E, d);

        if d == 7.5e-4
            fname = sprintf('%s/E_%g_d_%.1e.txt', datadir, E, d);
        endif

        % Load numeric data (9 columns)
        data = load(fname);

        time          = data(:,1);
        timestep      = data(:,2);
        iterations    = data(:,3);
        max_condition = data(:,4);
        last_condition= data(:,5);
        max_e0        = data(:,6);
        mean_e0       = data(:,7);
        max_A0        = data(:,8);
        mean_A0       = data(:,9);

        % Store in struct
        key = sprintf('E_%g_d_%g', E, d);
        results.(key).time = time;
        results.(key).iterations = iterations;
        results.(key).max_condition = max_condition;
        results.(key).last_condition = last_condition;
        results.(key).max_e0 = max_e0;
        results.(key).mean_e0 = mean_e0;
        results.(key).max_A0 = max_A0;
        results.(key).mean_A0 = mean_A0;

        % === Make 5 plots per file ===
        %figure; plot(time, iterations);
        %xlabel('time'); ylabel('iterations');
        %title(sprintf('Iterations vs time (E=%g, d=%g)', E, d));

        %figure; plot(time, max_condition);
        %xlabel('time'); ylabel('max condition');
        %title(sprintf('Max condition vs time (E=%g, d=%g)', E, d));

        %figure; plot(time, last_condition);
        %xlabel('time'); ylabel('last condition');
        %title(sprintf('Last condition vs time (E=%g, d=%g)', E, d));

        %figure; plot(time, mean_e0, time, max_e0);
        %xlabel('time'); ylabel('e0');
        %legend('mean','max'); legend('boxoff');
        %title(sprintf('e0 vs time (E=%g, d=%g)', E, d));

        %figure; plot(time, mean_A0, time, max_A0);
        %xlabel('time'); ylabel('A0');
        %legend('mean','max'); legend('boxoff');
        %title(sprintf('A0 vs time (E=%g, d=%g)', E, d));

        % === Compute averages over file ===
        results.(key).mean_iterations    = mean(iterations);
        results.(key).mean_max_condition = mean(max_condition);
        results.(key).mean_last_condition= mean(last_condition);
    end
end

% === Collect averages into arrays ===
mean_iter = zeros(numel(E_list), numel(delta_list));
mean_maxc = zeros(numel(E_list), numel(delta_list));
mean_lastc= zeros(numel(E_list), numel(delta_list));

for Ei = 1:numel(E_list)
    for di = 1:numel(delta_list)
        E = E_list(Ei);
        d = delta_list(di);
        key = sprintf('E_%g_d_%g', E, d);
        mean_iter(Ei,di) = results.(key).mean_iterations;
        mean_maxc(Ei,di) = results.(key).mean_max_condition;
        mean_lastc(Ei,di)= results.(key).mean_last_condition;
    end
end

% === Summary plots across E ===

% Mean iterations vs E
figure("position", [100, 100, 1920, 1080]); hold on;
for di = 1:numel(delta_list)
    semilogx(E_list, mean_iter(:,di), '-o', 'DisplayName', sprintf('delta=%.0e', delta_list(di)));
end
xlabel('E'); ylabel('Mean iterations');
h = legend('show'); legend('location','southeast');
set (h, "fontsize", 10);
title('Mean iterations vs E');
box on;
print(sprintf("%s/mean_iterations.png", datadir), "-S1920,1080", "-r600", "-dpng")

% Mean max condition vs E
figure("position", [100, 100, 1920, 1080]); hold on;
for di = 1:numel(delta_list)
    loglog(E_list, mean_maxc(:,di), '-o', 'DisplayName', sprintf('delta=%.0e', delta_list(di)));
end
xlabel('E'); ylabel('Mean condition number.');
h = legend('show'); legend('location','northwest');
set (h, "fontsize", 10);
title('Condition number measured at most ill-conditioned iteration.');
box on;
print(sprintf("%s/mean_max_condition.png", datadir), "-S1920,1080", "-r600", "-dpng")

% Mean last condition vs E
figure("position", [100, 100, 1920, 1080]); hold on;
for di = 1:numel(delta_list)
    loglog(E_list, mean_lastc(:,di), '-o', 'DisplayName', sprintf('delta=%.0e', delta_list(di)));
end
xlabel('E'); ylabel('Mean condition number.');
h = legend('show'); legend('location','northwest');
set (h, "fontsize", 10);
title('Condition number measured at last iteration.');
box on;
print(sprintf("%s/mean_last_condition.png", datadir), "-S1920,1080", "-r600", "-dpng")







