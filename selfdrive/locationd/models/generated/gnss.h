#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5779776890182477754);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2524939468294332737);
void gnss_H_mod_fun(double *state, double *out_3532638154457308636);
void gnss_f_fun(double *state, double dt, double *out_5197086194607967569);
void gnss_F_fun(double *state, double dt, double *out_552397493946265820);
void gnss_h_6(double *state, double *sat_pos, double *out_3917269104953355206);
void gnss_H_6(double *state, double *sat_pos, double *out_4112407093749703676);
void gnss_h_20(double *state, double *sat_pos, double *out_3063995755499871504);
void gnss_H_20(double *state, double *sat_pos, double *out_2979466385522926301);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_119758281977513422);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7033002257333706942);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_119758281977513422);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7033002257333706942);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}