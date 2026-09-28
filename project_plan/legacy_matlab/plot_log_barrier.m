% Usage: octave plot_data.m data.txt
% The file must have columns: e vn N n dn

%args = argv();
%if (numel(args) < 1)
%    error("Usage: octave plot_data.m datafile.txt");
%end
%filename = args{1};
filename = "theta_1.txt";

% Load data
data = load(filename);
if (size(data, 2) < 4)
    error("Expected at least 5 columns: vn N n dn");
end

vn = data(:,1);
N  = data(:,2);
n  = data(:,3);
dn = data(:,4);

% Style settings
line_width   = 2;
marker_size  = 8;
font_size    = 20;
title_size   = 30;

figure('Position', [100, 100, 1920, 1080]); % Bigger figure



% --- dt*vn vs N, n, dn ---
subplot(3,1,1);
plot(vn, N, '-', 'LineWidth', line_width, 'MarkerSize', marker_size);
xlabel('vn', 'FontSize', font_size);
ylabel('N', 'FontSize', font_size);
title('vn vs N', 'FontSize', title_size);
grid on;

subplot(3,1,2);
plot(vn, n, '-', 'LineWidth', line_width, 'MarkerSize', marker_size);
xlabel('vn', 'FontSize', font_size);
ylabel('n', 'FontSize', font_size);
title('vn vs n', 'FontSize', title_size);
grid on;

subplot(3,1,3);
plot(vn, dn, '-', 'LineWidth', line_width, 'MarkerSize', marker_size);
xlabel('vn', 'FontSize', font_size);
ylabel('dn', 'FontSize', font_size);
title('vn vs dn', 'FontSize', title_size);
grid on;

% Apply font size to all axes
set(findall(gcf,'-property','FontSize'),'FontSize',font_size);
