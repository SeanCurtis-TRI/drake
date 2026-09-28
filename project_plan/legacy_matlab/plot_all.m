set(0, 'DefaultAxesFontSize', 18);
set(0, 'DefaultTextFontSize', 18);

data = dlmread('data.txt', '', 1, 0);

e       = data(:,1);
N       = data(:,3);
n       = data(:,4);
dn      = data(:,5);
n_reg   = data(:,6);
dn_reg  = data(:,7);

figure('Name','N vs e','NumberTitle','off');
plot(e, N, '-'); xlabel('e'); ylabel('N'); grid on; title('N vs e');

figure('Name','n vs e','NumberTitle','off');
plot(e, n, '-'); xlabel('e'); ylabel('n'); grid on; title('n vs e');

figure('Name','dn vs e','NumberTitle','off');
plot(e, dn, '-'); xlabel('e'); ylabel('dn'); grid on; title('dn vs e');

figure('Name','n\_reg vs e','NumberTitle','off');
plot(e, n_reg, '-'); xlabel('e'); ylabel('n\_reg'); grid on; title('n\_reg vs e');

figure('Name','dn\_reg vs e','NumberTitle','off');
plot(e, dn_reg, '-'); xlabel('e'); ylabel('dn\_reg'); grid on; title('dn\_reg vs e');
