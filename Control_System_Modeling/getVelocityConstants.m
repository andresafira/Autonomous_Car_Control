function [Kp, Kff] = getVelocityConstants(m, tau, b)
Kff = b;
Kv = m / tau - b;
end
