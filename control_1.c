#define S_FUNCTION_NAME control_1
#define S_FUNCTION_LEVEL 2

#define TPWM Tsampling //Tpwm en seg

#include "simstruc.h"
#define Tsampling 1.0 / 2000.0

// BEGIN Globals
#include <math.h>
#include "lib/ffm_2d_modulator.h"

float V_hat_l_1;
float P_ref_i_l_1;
float P_hat_l_1;
float i_hat_l_1;

// END

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!ssSetNumInputPorts(S, 4)) return;
    //ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
    //↓ una por entrada s, salida, dimensión
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortWidth(S, 2, 2);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortWidth(S, 3, 2);
    ssSetInputPortDirectFeedThrough(S, 3, 1);


    if (!ssSetNumOutputPorts(S,1)) return;
    //ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetOutputPortWidth(S, 0, 2);

    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    //ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    // Tasa de muestreo ↓
    ssSetSampleTime(S, 0, Tsampling);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S) {
    V_hat_l_1 = 0;
    P_ref_i_l_1 = 0;
    i_hat_l_1 = 0;
    P_hat_l_1 = 0;
}
#endif

static void mdlOutputs(SimStruct *S, int_T tid)
{

    // BEGIN Declarations

    InputRealPtrsType pV_grid = ssGetInputPortRealSignalPtrs(S, 0);
    InputRealPtrsType pI_ab = ssGetInputPortRealSignalPtrs(S, 1);
    InputRealPtrsType pV_cond = ssGetInputPortRealSignalPtrs(S, 2);
    InputRealPtrsType pV_cond_ref = ssGetInputPortRealSignalPtrs(S,3);

    real_T *y1 = ssGetOutputPortRealSignal(S, 0);

    // END


    // BEGIN Assignation

    float V_grid, I_ab, V_c1, V_c2, V_c1_ref, V_c2_ref, P_dc_ref, P_dc;
    V_grid = *pV_grid[0];
    I_ab = *pI_ab[0];
    V_c1 = *pV_cond[0];
    V_c2 = *pV_cond[1];
    V_c1_ref = *pV_cond_ref[0];
    V_c2_ref = * pV_cond_ref[1];
    P_dc = pow(V_c1 + V_c2, 2);
    P_dc_ref = pow(V_c1_ref + V_c2_ref, 2);

    // END

    // BEGIN Outer

    float P_ref_i;
    float P_hat = P_dc_ref - P_dc;
    float Kp_outer, Ki_outer;
    Kp_outer = 1.0 / 50.0;
    Ki_outer = 5;
    P_ref_i = Kp_outer * P_hat + (Ki_outer * Tsampling - Kp_outer) * P_hat_l_1 + P_ref_i_l_1;
    P_ref_i_l_1 = P_ref_i;
    P_hat_l_1 = P_hat;

    // END

    // BEGIN Transition

    float i_hat;
    i_hat = - P_ref_i * V_grid / pow(230, 2) - I_ab;

    // END

    // BEGIN Inner

    float Kp_inner, Ki_inner;
    float V_hat;
    float V_ab_ref;
    Kp_inner = 5;
    Ki_inner =  0;
    V_hat = Kp_inner * i_hat + (Ki_inner * Tsampling - Kp_inner) * i_hat_l_1 + V_hat_l_1;
    V_hat_l_1 = V_hat;
    i_hat_l_1 = i_hat;
    V_ab_ref = (-V_hat + V_grid);

    // END Inner

    // BEGIN Modulator

    ffm_2d_args args = {
        .I_ab = I_ab,
        .V_ab_ref = V_ab_ref,
        .V_c1 = V_c1,
        .V_c2 = V_c2,
        .V_c1_ref = V_c1_ref,
        .V_c2_ref = V_c2_ref,
    };
    ffm_2d_res res = ffm_2d(args);

    // END

    // BEGIN Output
    y1[0] = res.delta_u;
    y1[1] = res.delta_l;
    // END
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
