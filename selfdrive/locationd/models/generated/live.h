#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_2385028492105558594);
void live_err_fun(double *nom_x, double *delta_x, double *out_6753072506514245053);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_297723459863873139);
void live_H_mod_fun(double *state, double *out_1777158878569955168);
void live_f_fun(double *state, double dt, double *out_8962073409035415999);
void live_F_fun(double *state, double dt, double *out_6300143064168835345);
void live_h_4(double *state, double *unused, double *out_4894791780580602543);
void live_H_4(double *state, double *unused, double *out_7395350518109986198);
void live_h_9(double *state, double *unused, double *out_741181868362849977);
void live_H_9(double *state, double *unused, double *out_7636540164739576843);
void live_h_10(double *state, double *unused, double *out_3431684253627843692);
void live_H_10(double *state, double *unused, double *out_4270402486335561987);
void live_h_12(double *state, double *unused, double *out_1149378860520808245);
void live_H_12(double *state, double *unused, double *out_6031937147567603623);
void live_h_35(double *state, double *unused, double *out_8415562861965906932);
void live_H_35(double *state, double *unused, double *out_7684731498226958042);
void live_h_32(double *state, double *unused, double *out_3693683631025888891);
void live_H_32(double *state, double *unused, double *out_7875531152634139078);
void live_h_13(double *state, double *unused, double *out_4340664960337158482);
void live_H_13(double *state, double *unused, double *out_7917615147219878690);
void live_h_14(double *state, double *unused, double *out_741181868362849977);
void live_H_14(double *state, double *unused, double *out_7636540164739576843);
void live_h_33(double *state, double *unused, double *out_372088024056581870);
void live_H_33(double *state, double *unused, double *out_4534174493588100438);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}