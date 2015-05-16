#ifndef __c2_ARDrone2_DiscreteSimulation_h__
#define __c2_ARDrone2_DiscreteSimulation_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_ARDrone2_DiscreteSimulationInstanceStruct
#define typedef_SFc2_ARDrone2_DiscreteSimulationInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_ARDrone2_DiscreteSimulation;
  real_T *c2_V_xy_n;
  real_T *c2_V_z;
  real_T *c2_omega_psi_n;
  real_T *c2_x_ref_n;
  real_T *c2_y_ref_n;
  real_T *c2_z_ref_n;
  real_T *c2_x_n;
  real_T *c2_y_n;
  real_T *c2_z_n;
  real_T *c2_psi_nm1;
  real_T *c2_psi_ez_nm1;
  real_T *c2_t;
  real_T *c2_psi_ez_n;
} SFc2_ARDrone2_DiscreteSimulationInstanceStruct;

#endif                                 /*typedef_SFc2_ARDrone2_DiscreteSimulationInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c2_ARDrone2_DiscreteSimulation_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_ARDrone2_DiscreteSimulation_get_check_sum(mxArray *plhs[]);
extern void c2_ARDrone2_DiscreteSimulation_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
