function [Gx, Gphi] = encontrarGPos(Mp, tr, v, L)

s = tf('s');
xi = -log(Mp) / sqrt(pi^2 + log(Mp)^2);
wn = (pi - acos(xi)) / (tr * sqrt(1 - xi^2)); 

Ktheta = 2 * xi * wn;
Kx = wn^2 / (v * Ktheta);

Gx = Kx * Ktheta * v / (s^2 + s * Ktheta + Kx * Ktheta * v);

Gphi = (Kx - Gx * Kx - Gx * s / v) * Ktheta * L / v;

end