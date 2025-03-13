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
void car_err_fun(double *nom_x, double *delta_x, double *out_3037610658734927841);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_810506188514175946);
void car_H_mod_fun(double *state, double *out_5984654964435016815);
void car_f_fun(double *state, double dt, double *out_648400194143585294);
void car_F_fun(double *state, double dt, double *out_2341512587529460492);
void car_h_25(double *state, double *unused, double *out_3336978377673773743);
void car_H_25(double *state, double *unused, double *out_8137211606712700714);
void car_h_24(double *state, double *unused, double *out_1832312198493549389);
void car_H_24(double *state, double *unused, double *out_5563023140030341868);
void car_h_30(double *state, double *unused, double *out_8338930469685426937);
void car_H_30(double *state, double *unused, double *out_1220521265221083959);
void car_h_26(double *state, double *unused, double *out_9164303751122462244);
void car_H_26(double *state, double *unused, double *out_6568029148122794678);
void car_h_27(double *state, double *unused, double *out_1664694889987071902);
void car_H_27(double *state, double *unused, double *out_3395284577021508870);
void car_h_29(double *state, double *unused, double *out_1939888952271577791);
void car_H_29(double *state, double *unused, double *out_710289920906691775);
void car_h_28(double *state, double *unused, double *out_2268010774663549419);
void car_H_28(double *state, double *unused, double *out_5792688937976222349);
void car_h_31(double *state, double *unused, double *out_4426286879146898698);
void car_H_31(double *state, double *unused, double *out_8106565644835740286);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}