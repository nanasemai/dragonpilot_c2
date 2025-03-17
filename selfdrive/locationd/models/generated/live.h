#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1335610085814845024);
void live_err_fun(double *nom_x, double *delta_x, double *out_4445248801594642706);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2255140372509373603);
void live_H_mod_fun(double *state, double *out_1108067141344716762);
void live_f_fun(double *state, double dt, double *out_140646063848144387);
void live_F_fun(double *state, double dt, double *out_3554679380087543391);
void live_h_4(double *state, double *unused, double *out_3487048564883728180);
void live_H_4(double *state, double *unused, double *out_150477020404018729);
void live_h_9(double *state, double *unused, double *out_1921223185975565916);
void live_H_9(double *state, double *unused, double *out_391666667033609374);
void live_h_10(double *state, double *unused, double *out_3354837286932339813);
void live_H_10(double *state, double *unused, double *out_4894359973852654635);
void live_h_12(double *state, double *unused, double *out_5289906301413594232);
void live_H_12(double *state, double *unused, double *out_5169933428435980524);
void live_h_35(double *state, double *unused, double *out_7591857235898169327);
void live_H_35(double *state, double *unused, double *out_3517139077776626105);
void live_h_32(double *state, double *unused, double *out_1296020976724731790);
void live_H_32(double *state, double *unused, double *out_3397725425917490251);
void live_h_13(double *state, double *unused, double *out_8312385215584114470);
void live_H_13(double *state, double *unused, double *out_3207808226006723392);
void live_h_14(double *state, double *unused, double *out_1921223185975565916);
void live_H_14(double *state, double *unused, double *out_391666667033609374);
void live_h_33(double *state, double *unused, double *out_2757221046863243653);
void live_H_33(double *state, double *unused, double *out_6667696082415483709);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}