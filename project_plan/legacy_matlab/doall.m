R = 0.1;
E = 0.5*1e8;
m = 0.1;
H = 3;
g = 9.81;
delta = 1e-4;

p.R = R;
p.E = E;
p.delta = delta;
p.m = m;
p.g =g;
p.h = H;

target = @(x) log_energy_target(x, p);

x = fzero(target, 0.99*delta)
e = x/(2*delta)
