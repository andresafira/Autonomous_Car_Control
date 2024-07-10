function [Gv, Gf] = encontrarGVelocidade(m, tau, b)

s = tf('s');
Kv = m / tau - b;
Gv = (Kv + b) / (m*s + b + Kv);

Gf = Kv - Gv * Kv + b;

end