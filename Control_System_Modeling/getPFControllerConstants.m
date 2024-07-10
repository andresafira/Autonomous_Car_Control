function [Kx, Ktheta] = getPFControllerConstants(Mp, tr, v, L)
xi = -log(Mp) / sqrt(pi^2 + log(Mp)^2);
wn = (pi - acos(xi)) / (tr * sqrt(1 - xi^2)); 

Ktheta = 2 * xi * wn;
Kx = wn^2 / (v * Ktheta);
end
