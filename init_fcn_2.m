mex control_2.c lib/ffm_3d_modulator.c;

Tint = 1e-6;
Ts = 1/2000;

L_s = 5e-3;
V_source = 230 * sqrt(2);
R_load = 10;
C_load = 5e-2;
C_init_v = 200;

shift = (180/3)/360 * Ts;