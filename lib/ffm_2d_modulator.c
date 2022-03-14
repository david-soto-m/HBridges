#include "ffm_2d_modulator.h"

typedef struct Ffm_2d_Control {
    const float K_p;
    const float K_i;
    float epsilon_k_l_1;
} ffm_2d_control;

ffm_2d_control ffm_2d_cont = {
    .K_p = 1.0 / 100.0,
    .K_i = 1,
    .epsilon_k_l_1 = 0
};


ffm_2d_res ffm_2d(ffm_2d_args args) {
    ffm_2d_res res;

    // BEGIN Equilibrium
    float eq_x, eq_y;
    float half_V = args.V_ab_ref / 2.0;
    if (half_V > args.V_c2) {
        eq_x = args.V_c2;
        eq_y = args.V_ab_ref - args.V_c2;
    } else if (half_V > args.V_c1) {
        eq_x = args.V_ab_ref - args.V_c1;
        eq_y = args.V_c1;
    } else if (half_V < -args.V_c2) {
        eq_x = - args.V_c2;
        eq_y = args.V_ab_ref + args.V_c2;
    } else if (half_V < -args.V_c1) {
        eq_x = args.V_ab_ref + args.V_c1;
        eq_y = - args.V_c1;
    } else {
        eq_x = half_V;
        eq_y = half_V;
    }
    // END Equilibrium

    // BEGIN Correction
    float D_V_c1, D_V_c2, epsilon_k, chi_k;
    float delta_low, delta_up;
    D_V_c1 = args.V_c1_ref - args.V_c1;
    D_V_c2 = args.V_c2_ref - args.V_c2;
    epsilon_k = (D_V_c1 - D_V_c2) * args.I_ab;
    chi_k = (epsilon_k + ffm_2d_cont.epsilon_k_l_1) / 2.0;
    delta_up = eq_y + ffm_2d_cont.K_p * epsilon_k + ffm_2d_cont.K_i * chi_k;
    delta_low = args.V_ab_ref - delta_up;
    ffm_2d_cont.epsilon_k_l_1 = epsilon_k;
    // END Correction

    res.delta_l = delta_low / args.V_c2_ref;
    res.delta_u = delta_up / args.V_c1_ref;

    return res;
}

