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
void car_err_fun(double *nom_x, double *delta_x, double *out_4362128129652044840);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6130845242579364597);
void car_H_mod_fun(double *state, double *out_1914720213026091055);
void car_f_fun(double *state, double dt, double *out_3890178460933007454);
void car_F_fun(double *state, double dt, double *out_7489115981160127478);
void car_h_25(double *state, double *unused, double *out_5935470540770750454);
void car_H_25(double *state, double *unused, double *out_6170791130760140851);
void car_h_24(double *state, double *unused, double *out_5175324518545799938);
void car_H_24(double *state, double *unused, double *out_8343440729765640417);
void car_h_30(double *state, double *unused, double *out_6393671902929312116);
void car_H_30(double *state, double *unused, double *out_3652458172252892224);
void car_h_26(double *state, double *unused, double *out_5450162050027649681);
void car_H_26(double *state, double *unused, double *out_8534449624075354541);
void car_h_27(double *state, double *unused, double *out_1537518459489121442);
void car_H_27(double *state, double *unused, double *out_5827221484053317135);
void car_h_29(double *state, double *unused, double *out_9016286119335906815);
void car_H_29(double *state, double *unused, double *out_3142226827938500040);
void car_h_28(double *state, double *unused, double *out_3464154808171724203);
void car_H_28(double *state, double *unused, double *out_8224625845008030614);
void car_h_31(double *state, double *unused, double *out_5778283872419621309);
void car_H_31(double *state, double *unused, double *out_7908241521842003065);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}