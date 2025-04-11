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
void live_H(double *in_vec, double *out_2562738696775388398);
void live_err_fun(double *nom_x, double *delta_x, double *out_5345183143111557466);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8132837573948108574);
void live_H_mod_fun(double *state, double *out_6641978206587233878);
void live_f_fun(double *state, double dt, double *out_6315024426537972650);
void live_F_fun(double *state, double dt, double *out_1101455076561017260);
void live_h_4(double *state, double *unused, double *out_2409560230227215179);
void live_H_4(double *state, double *unused, double *out_3792514916460275118);
void live_h_9(double *state, double *unused, double *out_5563274265529299809);
void live_H_9(double *state, double *unused, double *out_3494704018804172352);
void live_h_10(double *state, double *unused, double *out_7154826775933599249);
void live_H_10(double *state, double *unused, double *out_89714083669468213);
void live_h_12(double *state, double *unused, double *out_8102425060769564980);
void live_H_12(double *state, double *unused, double *out_3874613397222175374);
void live_h_35(double *state, double *unused, double *out_2973163444656156158);
void live_H_35(double *state, double *unused, double *out_6620176429547189083);
void live_h_32(double *state, double *unused, double *out_9209196625270152995);
void live_H_32(double *state, double *unused, double *out_6025263271954749975);
void live_h_13(double *state, double *unused, double *out_721120747884715016);
void live_H_13(double *state, double *unused, double *out_943452109880212401);
void live_h_14(double *state, double *unused, double *out_5563274265529299809);
void live_H_14(double *state, double *unused, double *out_3494704018804172352);
void live_h_33(double *state, double *unused, double *out_8060901591285736869);
void live_H_33(double *state, double *unused, double *out_8676010639523504929);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}