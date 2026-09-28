clear;
close all;

filename = 'data.txt';
data = readmatrix(filename);

t      = data(:,1);
h      = data(:,2);
min_e  = data(:,3);
max_e  = data(:,4);
mean_e = data(:,5);
surface_area = data(:,6);

figure;
semilogy(t, h, 'o-');
xlabel('t');
ylabel('h');
title('t vs h');

figure;
plot(t, min_e, 'o-', t, max_e, 'x-', t, mean_e, '-');
xlabel('t');
ylabel('h');
title('t vs e');
legend('min(e)', 'max(e)', 'mean(e)');

figure;
plot(t, surface_area, 'o-');
xlabel('t');
ylabel('F [N]');
title('t vs Area');

areas = readmatrix('areas.txt');
figure;
hist(areas, 100);
