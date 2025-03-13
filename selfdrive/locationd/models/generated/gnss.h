#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6869195426061586587);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4824792627629565069);
void gnss_H_mod_fun(double *state, double *out_1874210704324695298);
void gnss_f_fun(double *state, double dt, double *out_2252706745413448914);
void gnss_F_fun(double *state, double dt, double *out_6359409139098178053);
void gnss_h_6(double *state, double *sat_pos, double *out_6178478711282234389);
void gnss_H_6(double *state, double *sat_pos, double *out_3886623360167537523);
void gnss_h_20(double *state, double *sat_pos, double *out_6589641369073008610);
void gnss_H_20(double *state, double *sat_pos, double *out_5831881944237969506);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1986343695071145740);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2321963168967561661);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1986343695071145740);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2321963168967561661);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}