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
void car_err_fun(double *nom_x, double *delta_x, double *out_1443315500284732058);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2873127258815959395);
void car_H_mod_fun(double *state, double *out_2638827299528522109);
void car_f_fun(double *state, double dt, double *out_7417560445590253104);
void car_F_fun(double *state, double dt, double *out_4097394295836897863);
void car_h_25(double *state, double *unused, double *out_989922971562397546);
void car_H_25(double *state, double *unused, double *out_9113945678895749699);
void car_h_24(double *state, double *unused, double *out_743147310726708258);
void car_H_24(double *state, double *unused, double *out_5774872796011478781);
void car_h_30(double *state, double *unused, double *out_4733599054057522454);
void car_H_30(double *state, double *unused, double *out_6814465436306553290);
void car_h_26(double *state, double *unused, double *out_8711471751991183517);
void car_H_26(double *state, double *unused, double *out_5372442360021693475);
void car_h_27(double *state, double *unused, double *out_7388964488976246692);
void car_H_27(double *state, double *unused, double *out_6809843419952084718);
void car_h_29(double *state, double *unused, double *out_4286944358382953988);
void car_H_29(double *state, double *unused, double *out_6304234091992161106);
void car_h_28(double *state, double *unused, double *out_8556708290052388429);
void car_H_28(double *state, double *unused, double *out_7060110964647859936);
void car_h_31(double *state, double *unused, double *out_4851606447990899198);
void car_H_31(double *state, double *unused, double *out_4746234257788341999);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}