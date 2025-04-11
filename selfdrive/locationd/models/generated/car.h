#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4570434651515648313);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8600933468682283131);
void car_H_mod_fun(double *state, double *out_6112867613499914667);
void car_f_fun(double *state, double dt, double *out_6564035153870315201);
void car_F_fun(double *state, double dt, double *out_1314263995452588577);
void car_h_25(double *state, double *unused, double *out_7367858087962412775);
void car_H_25(double *state, double *unused, double *out_2198594579725619921);
void car_h_24(double *state, double *unused, double *out_7535299805146712907);
void car_H_24(double *state, double *unused, double *out_6186650779258166512);
void car_h_30(double *state, double *unused, double *out_7643052150246918664);
void car_H_30(double *state, double *unused, double *out_6726290909853228119);
void car_h_26(double *state, double *unused, double *out_8423520550828414603);
void car_H_26(double *state, double *unused, double *out_5940097898599676145);
void car_h_27(double *state, double *unused, double *out_5200600498188423753);
void car_H_27(double *state, double *unused, double *out_4502696838669284902);
void car_h_29(double *state, double *unused, double *out_762604669397307989);
void car_H_29(double *state, double *unused, double *out_6216059565538835935);
void car_h_28(double *state, double *unused, double *out_198927230527578108);
void car_H_28(double *state, double *unused, double *out_7148285491101185107);
void car_h_31(double *state, double *unused, double *out_2582225304483398849);
void car_H_31(double *state, double *unused, double *out_2167948617848659493);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}