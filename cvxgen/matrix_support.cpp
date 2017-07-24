/* Produced by CVXGEN, 2017-07-24 04:29:55 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = 0;
  lhs[12] = 0;
  lhs[13] = 0;
  lhs[14] = 0;
  lhs[15] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.J[0])-rhs[1]*(params.J[9])-rhs[2]*(params.J[18])-rhs[3]*(params.J[27])-rhs[4]*(params.J[36])-rhs[5]*(params.J[45])-rhs[6]*(params.J[54])-rhs[7]*(-1);
  lhs[1] = -rhs[0]*(params.J[1])-rhs[1]*(params.J[10])-rhs[2]*(params.J[19])-rhs[3]*(params.J[28])-rhs[4]*(params.J[37])-rhs[5]*(params.J[46])-rhs[6]*(params.J[55])-rhs[8]*(-1);
  lhs[2] = -rhs[0]*(params.J[2])-rhs[1]*(params.J[11])-rhs[2]*(params.J[20])-rhs[3]*(params.J[29])-rhs[4]*(params.J[38])-rhs[5]*(params.J[47])-rhs[6]*(params.J[56])-rhs[9]*(-1);
  lhs[3] = -rhs[0]*(params.J[3])-rhs[1]*(params.J[12])-rhs[2]*(params.J[21])-rhs[3]*(params.J[30])-rhs[4]*(params.J[39])-rhs[5]*(params.J[48])-rhs[6]*(params.J[57])-rhs[10]*(-1);
  lhs[4] = -rhs[0]*(params.J[4])-rhs[1]*(params.J[13])-rhs[2]*(params.J[22])-rhs[3]*(params.J[31])-rhs[4]*(params.J[40])-rhs[5]*(params.J[49])-rhs[6]*(params.J[58])-rhs[11]*(-1);
  lhs[5] = -rhs[0]*(params.J[5])-rhs[1]*(params.J[14])-rhs[2]*(params.J[23])-rhs[3]*(params.J[32])-rhs[4]*(params.J[41])-rhs[5]*(params.J[50])-rhs[6]*(params.J[59])-rhs[12]*(-1);
  lhs[6] = -rhs[0]*(params.J[6])-rhs[1]*(params.J[15])-rhs[2]*(params.J[24])-rhs[3]*(params.J[33])-rhs[4]*(params.J[42])-rhs[5]*(params.J[51])-rhs[6]*(params.J[60])-rhs[13]*(-1);
  lhs[7] = -rhs[0]*(params.J[7])-rhs[1]*(params.J[16])-rhs[2]*(params.J[25])-rhs[3]*(params.J[34])-rhs[4]*(params.J[43])-rhs[5]*(params.J[52])-rhs[6]*(params.J[61])-rhs[14]*(-1);
  lhs[8] = -rhs[0]*(params.J[8])-rhs[1]*(params.J[17])-rhs[2]*(params.J[26])-rhs[3]*(params.J[35])-rhs[4]*(params.J[44])-rhs[5]*(params.J[53])-rhs[6]*(params.J[62])-rhs[15]*(-1);
  lhs[9] = -rhs[0]*(-params.J[0])-rhs[1]*(-params.J[9])-rhs[2]*(-params.J[18])-rhs[3]*(-params.J[27])-rhs[4]*(-params.J[36])-rhs[5]*(-params.J[45])-rhs[6]*(-params.J[54])-rhs[7]*(-1);
  lhs[10] = -rhs[0]*(-params.J[1])-rhs[1]*(-params.J[10])-rhs[2]*(-params.J[19])-rhs[3]*(-params.J[28])-rhs[4]*(-params.J[37])-rhs[5]*(-params.J[46])-rhs[6]*(-params.J[55])-rhs[8]*(-1);
  lhs[11] = -rhs[0]*(-params.J[2])-rhs[1]*(-params.J[11])-rhs[2]*(-params.J[20])-rhs[3]*(-params.J[29])-rhs[4]*(-params.J[38])-rhs[5]*(-params.J[47])-rhs[6]*(-params.J[56])-rhs[9]*(-1);
  lhs[12] = -rhs[0]*(-params.J[3])-rhs[1]*(-params.J[12])-rhs[2]*(-params.J[21])-rhs[3]*(-params.J[30])-rhs[4]*(-params.J[39])-rhs[5]*(-params.J[48])-rhs[6]*(-params.J[57])-rhs[10]*(-1);
  lhs[13] = -rhs[0]*(-params.J[4])-rhs[1]*(-params.J[13])-rhs[2]*(-params.J[22])-rhs[3]*(-params.J[31])-rhs[4]*(-params.J[40])-rhs[5]*(-params.J[49])-rhs[6]*(-params.J[58])-rhs[11]*(-1);
  lhs[14] = -rhs[0]*(-params.J[5])-rhs[1]*(-params.J[14])-rhs[2]*(-params.J[23])-rhs[3]*(-params.J[32])-rhs[4]*(-params.J[41])-rhs[5]*(-params.J[50])-rhs[6]*(-params.J[59])-rhs[12]*(-1);
  lhs[15] = -rhs[0]*(-params.J[6])-rhs[1]*(-params.J[15])-rhs[2]*(-params.J[24])-rhs[3]*(-params.J[33])-rhs[4]*(-params.J[42])-rhs[5]*(-params.J[51])-rhs[6]*(-params.J[60])-rhs[13]*(-1);
  lhs[16] = -rhs[0]*(-params.J[7])-rhs[1]*(-params.J[16])-rhs[2]*(-params.J[25])-rhs[3]*(-params.J[34])-rhs[4]*(-params.J[43])-rhs[5]*(-params.J[52])-rhs[6]*(-params.J[61])-rhs[14]*(-1);
  lhs[17] = -rhs[0]*(-params.J[8])-rhs[1]*(-params.J[17])-rhs[2]*(-params.J[26])-rhs[3]*(-params.J[35])-rhs[4]*(-params.J[44])-rhs[5]*(-params.J[53])-rhs[6]*(-params.J[62])-rhs[15]*(-1);
  lhs[18] = -rhs[0]*(-1);
  lhs[19] = -rhs[1]*(-1);
  lhs[20] = -rhs[2]*(-1);
  lhs[21] = -rhs[3]*(-1);
  lhs[22] = -rhs[4]*(-1);
  lhs[23] = -rhs[5]*(-1);
  lhs[24] = -rhs[6]*(-1);
  lhs[25] = -rhs[0]*(1);
  lhs[26] = -rhs[1]*(1);
  lhs[27] = -rhs[2]*(1);
  lhs[28] = -rhs[3]*(1);
  lhs[29] = -rhs[4]*(1);
  lhs[30] = -rhs[5]*(1);
  lhs[31] = -rhs[6]*(1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.J[0])-rhs[1]*(params.J[1])-rhs[2]*(params.J[2])-rhs[3]*(params.J[3])-rhs[4]*(params.J[4])-rhs[5]*(params.J[5])-rhs[6]*(params.J[6])-rhs[7]*(params.J[7])-rhs[8]*(params.J[8])-rhs[9]*(-params.J[0])-rhs[10]*(-params.J[1])-rhs[11]*(-params.J[2])-rhs[12]*(-params.J[3])-rhs[13]*(-params.J[4])-rhs[14]*(-params.J[5])-rhs[15]*(-params.J[6])-rhs[16]*(-params.J[7])-rhs[17]*(-params.J[8])-rhs[18]*(-1)-rhs[25]*(1);
  lhs[1] = -rhs[0]*(params.J[9])-rhs[1]*(params.J[10])-rhs[2]*(params.J[11])-rhs[3]*(params.J[12])-rhs[4]*(params.J[13])-rhs[5]*(params.J[14])-rhs[6]*(params.J[15])-rhs[7]*(params.J[16])-rhs[8]*(params.J[17])-rhs[9]*(-params.J[9])-rhs[10]*(-params.J[10])-rhs[11]*(-params.J[11])-rhs[12]*(-params.J[12])-rhs[13]*(-params.J[13])-rhs[14]*(-params.J[14])-rhs[15]*(-params.J[15])-rhs[16]*(-params.J[16])-rhs[17]*(-params.J[17])-rhs[19]*(-1)-rhs[26]*(1);
  lhs[2] = -rhs[0]*(params.J[18])-rhs[1]*(params.J[19])-rhs[2]*(params.J[20])-rhs[3]*(params.J[21])-rhs[4]*(params.J[22])-rhs[5]*(params.J[23])-rhs[6]*(params.J[24])-rhs[7]*(params.J[25])-rhs[8]*(params.J[26])-rhs[9]*(-params.J[18])-rhs[10]*(-params.J[19])-rhs[11]*(-params.J[20])-rhs[12]*(-params.J[21])-rhs[13]*(-params.J[22])-rhs[14]*(-params.J[23])-rhs[15]*(-params.J[24])-rhs[16]*(-params.J[25])-rhs[17]*(-params.J[26])-rhs[20]*(-1)-rhs[27]*(1);
  lhs[3] = -rhs[0]*(params.J[27])-rhs[1]*(params.J[28])-rhs[2]*(params.J[29])-rhs[3]*(params.J[30])-rhs[4]*(params.J[31])-rhs[5]*(params.J[32])-rhs[6]*(params.J[33])-rhs[7]*(params.J[34])-rhs[8]*(params.J[35])-rhs[9]*(-params.J[27])-rhs[10]*(-params.J[28])-rhs[11]*(-params.J[29])-rhs[12]*(-params.J[30])-rhs[13]*(-params.J[31])-rhs[14]*(-params.J[32])-rhs[15]*(-params.J[33])-rhs[16]*(-params.J[34])-rhs[17]*(-params.J[35])-rhs[21]*(-1)-rhs[28]*(1);
  lhs[4] = -rhs[0]*(params.J[36])-rhs[1]*(params.J[37])-rhs[2]*(params.J[38])-rhs[3]*(params.J[39])-rhs[4]*(params.J[40])-rhs[5]*(params.J[41])-rhs[6]*(params.J[42])-rhs[7]*(params.J[43])-rhs[8]*(params.J[44])-rhs[9]*(-params.J[36])-rhs[10]*(-params.J[37])-rhs[11]*(-params.J[38])-rhs[12]*(-params.J[39])-rhs[13]*(-params.J[40])-rhs[14]*(-params.J[41])-rhs[15]*(-params.J[42])-rhs[16]*(-params.J[43])-rhs[17]*(-params.J[44])-rhs[22]*(-1)-rhs[29]*(1);
  lhs[5] = -rhs[0]*(params.J[45])-rhs[1]*(params.J[46])-rhs[2]*(params.J[47])-rhs[3]*(params.J[48])-rhs[4]*(params.J[49])-rhs[5]*(params.J[50])-rhs[6]*(params.J[51])-rhs[7]*(params.J[52])-rhs[8]*(params.J[53])-rhs[9]*(-params.J[45])-rhs[10]*(-params.J[46])-rhs[11]*(-params.J[47])-rhs[12]*(-params.J[48])-rhs[13]*(-params.J[49])-rhs[14]*(-params.J[50])-rhs[15]*(-params.J[51])-rhs[16]*(-params.J[52])-rhs[17]*(-params.J[53])-rhs[23]*(-1)-rhs[30]*(1);
  lhs[6] = -rhs[0]*(params.J[54])-rhs[1]*(params.J[55])-rhs[2]*(params.J[56])-rhs[3]*(params.J[57])-rhs[4]*(params.J[58])-rhs[5]*(params.J[59])-rhs[6]*(params.J[60])-rhs[7]*(params.J[61])-rhs[8]*(params.J[62])-rhs[9]*(-params.J[54])-rhs[10]*(-params.J[55])-rhs[11]*(-params.J[56])-rhs[12]*(-params.J[57])-rhs[13]*(-params.J[58])-rhs[14]*(-params.J[59])-rhs[15]*(-params.J[60])-rhs[16]*(-params.J[61])-rhs[17]*(-params.J[62])-rhs[24]*(-1)-rhs[31]*(1);
  lhs[7] = -rhs[0]*(-1)-rhs[9]*(-1);
  lhs[8] = -rhs[1]*(-1)-rhs[10]*(-1);
  lhs[9] = -rhs[2]*(-1)-rhs[11]*(-1);
  lhs[10] = -rhs[3]*(-1)-rhs[12]*(-1);
  lhs[11] = -rhs[4]*(-1)-rhs[13]*(-1);
  lhs[12] = -rhs[5]*(-1)-rhs[14]*(-1);
  lhs[13] = -rhs[6]*(-1)-rhs[15]*(-1);
  lhs[14] = -rhs[7]*(-1)-rhs[16]*(-1);
  lhs[15] = -rhs[8]*(-1)-rhs[17]*(-1);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = 0;
  lhs[12] = 0;
  lhs[13] = 0;
  lhs[14] = 0;
  lhs[15] = 0;
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = 0;
  work.q[7] = 1;
  work.q[8] = 1;
  work.q[9] = 1;
  work.q[10] = 1;
  work.q[11] = 1;
  work.q[12] = 1;
  work.q[13] = 1;
  work.q[14] = 1;
  work.q[15] = 1;
}
void fillh(void) {
  work.h[0] = params.Xi[0];
  work.h[1] = params.Xi[1];
  work.h[2] = params.Xi[2];
  work.h[3] = params.Xi[3];
  work.h[4] = params.Xi[4];
  work.h[5] = params.Xi[5];
  work.h[6] = params.Xi[6];
  work.h[7] = params.Xi[7];
  work.h[8] = params.Xi[8];
  work.h[9] = -params.Xi[0];
  work.h[10] = -params.Xi[1];
  work.h[11] = -params.Xi[2];
  work.h[12] = -params.Xi[3];
  work.h[13] = -params.Xi[4];
  work.h[14] = -params.Xi[5];
  work.h[15] = -params.Xi[6];
  work.h[16] = -params.Xi[7];
  work.h[17] = -params.Xi[8];
  work.h[18] = -params.L_min[0];
  work.h[19] = -params.L_min[1];
  work.h[20] = -params.L_min[2];
  work.h[21] = -params.L_min[3];
  work.h[22] = -params.L_min[4];
  work.h[23] = -params.L_min[5];
  work.h[24] = -params.L_min[6];
  work.h[25] = params.L_max[0];
  work.h[26] = params.L_max[1];
  work.h[27] = params.L_max[2];
  work.h[28] = params.L_max[3];
  work.h[29] = params.L_max[4];
  work.h[30] = params.L_max[5];
  work.h[31] = params.L_max[6];
}
void fillb(void) {
}
void pre_ops(void) {
}
