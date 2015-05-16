#ifndef __c1_ARDrone2_Comparison_h__
#define __c1_ARDrone2_Comparison_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_ARDrone2_ComparisonInstanceStruct
#define typedef_SFc1_ARDrone2_ComparisonInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_ARDrone2_Comparison;
  real_T *c1_V_xy_n;
  real_T *c1_V_z;
  real_T *c1_omega_psi_n;
  real_T *c1_x_n;
  real_T *c1_y_n;
  real_T *c1_z_n;
  real_T *c1_psi_nm1;
  real_T *c1_psi_ez_nm1;
  real_T *c1_t;
  real_T *c1_psi_ez_n;
} SFc1_ARDrone2_ComparisonInstanceStruct;

#endif                                 /*typedef_SFc1_ARDrone2_ComparisonInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_ARDrone2_Comparison_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c1_ARDrone2_Comparison_get_check_sum(mxArray *plhs[]);
extern void c1_ARDrone2_Comparison_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
