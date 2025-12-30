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
void car_err_fun(double *nom_x, double *delta_x, double *out_3586577983331504426);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1775069159989754138);
void car_H_mod_fun(double *state, double *out_3154268613677279037);
void car_f_fun(double *state, double dt, double *out_1100623642042150356);
void car_F_fun(double *state, double dt, double *out_1716169941999921846);
void car_h_25(double *state, double *unused, double *out_6496658527081901115);
void car_H_25(double *state, double *unused, double *out_5519856422788446968);
void car_h_24(double *state, double *unused, double *out_6796312308294000941);
void car_H_24(double *state, double *unused, double *out_9077782021590770104);
void car_h_30(double *state, double *unused, double *out_8343538349365037331);
void car_H_30(double *state, double *unused, double *out_992160092660838770);
void car_h_26(double *state, double *unused, double *out_7066419672900181600);
void car_H_26(double *state, double *unused, double *out_1778353103914390744);
void car_h_27(double *state, double *unused, double *out_6948412278966804856);
void car_H_27(double *state, double *unused, double *out_3215754163844781987);
void car_h_29(double *state, double *unused, double *out_4000686288611319895);
void car_H_29(double *state, double *unused, double *out_1502391436975230954);
void car_h_28(double *state, double *unused, double *out_3672564466219348267);
void car_H_28(double *state, double *unused, double *out_3466021708540557205);
void car_h_31(double *state, double *unused, double *out_6221464464797395226);
void car_H_31(double *state, double *unused, double *out_5550502384665407396);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}