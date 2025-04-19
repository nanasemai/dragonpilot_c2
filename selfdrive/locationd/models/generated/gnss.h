#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3508247637958787068);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6010054649778819171);
void gnss_H_mod_fun(double *state, double *out_1212784004694500177);
void gnss_f_fun(double *state, double dt, double *out_6639798739313635028);
void gnss_F_fun(double *state, double dt, double *out_2149734763919352289);
void gnss_h_6(double *state, double *sat_pos, double *out_7203412672700125110);
void gnss_H_6(double *state, double *sat_pos, double *out_6921346676751383643);
void gnss_h_20(double *state, double *sat_pos, double *out_1679978481735855176);
void gnss_H_20(double *state, double *sat_pos, double *out_2805463512073601829);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3263449594829653104);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1997415651670555763);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3263449594829653104);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1997415651670555763);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}