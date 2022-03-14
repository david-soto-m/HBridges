#ifndef FFM_2D_MODULATOR
#define FFM_2D_MODULATOR

#include <math.h>

typedef struct Ffm_2d_Args {
    float V_ab_ref;
    float I_ab;
    float V_c1;
    float V_c1_ref;
    float V_c2;
    float V_c2_ref;
} ffm_2d_args;

typedef struct Ffm_2d_Res {
    float delta_l;
    float delta_u;
} ffm_2d_res;

ffm_2d_res ffm_2d(ffm_2d_args args);

#endif // FFM_2D_MODULATOR
