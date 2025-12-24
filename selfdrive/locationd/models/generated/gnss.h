#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5041675293586865981);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6156377347676432770);
void gnss_H_mod_fun(double *state, double *out_3166742716645354181);
void gnss_f_fun(double *state, double dt, double *out_3920457612290145952);
void gnss_F_fun(double *state, double dt, double *out_3799702480623368577);
void gnss_h_6(double *state, double *sat_pos, double *out_2435237834088609785);
void gnss_H_6(double *state, double *sat_pos, double *out_8702968871271221527);
void gnss_h_20(double *state, double *sat_pos, double *out_5218901198514814398);
void gnss_H_20(double *state, double *sat_pos, double *out_3631886714228571231);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8677941638066141178);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1473530525149698872);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8677941638066141178);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1473530525149698872);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}