#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3290531286296446277);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_1626950568301350913);
void gnss_H_mod_fun(double *state, double *out_3482262551659407560);
void gnss_f_fun(double *state, double dt, double *out_3240552207127151185);
void gnss_F_fun(double *state, double dt, double *out_4450169788449761540);
void gnss_h_6(double *state, double *sat_pos, double *out_6860602447084943242);
void gnss_H_6(double *state, double *sat_pos, double *out_2569914665657352674);
void gnss_h_20(double *state, double *sat_pos, double *out_7349164349304185710);
void gnss_H_20(double *state, double *sat_pos, double *out_324939251074071435);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3692256410067575061);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5030896998142331297);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3692256410067575061);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5030896998142331297);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}