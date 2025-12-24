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
void live_H(double *in_vec, double *out_5322837969451484780);
void live_err_fun(double *nom_x, double *delta_x, double *out_4878497025894929436);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4417997479367281059);
void live_H_mod_fun(double *state, double *out_8896299993676960161);
void live_f_fun(double *state, double dt, double *out_8824336041744460418);
void live_F_fun(double *state, double dt, double *out_3179030531582533576);
void live_h_4(double *state, double *unused, double *out_3329687987045659313);
void live_H_4(double *state, double *unused, double *out_6884611071756229505);
void live_h_9(double *state, double *unused, double *out_3424776059862686487);
void live_H_9(double *state, double *unused, double *out_6643421425126638860);
void live_h_10(double *state, double *unused, double *out_4249854668550369934);
void live_H_10(double *state, double *unused, double *out_4205728810657882815);
void live_h_12(double *state, double *unused, double *out_322731065412662763);
void live_H_12(double *state, double *unused, double *out_1865154663724267710);
void live_h_35(double *state, double *unused, double *out_5950071720235234386);
void live_H_35(double *state, double *unused, double *out_3517949014383622129);
void live_h_32(double *state, double *unused, double *out_2267591326129376707);
void live_H_32(double *state, double *unused, double *out_4153464520757209374);
void live_h_13(double *state, double *unused, double *out_3452940603325129625);
void live_H_13(double *state, double *unused, double *out_8033975084202764282);
void live_h_14(double *state, double *unused, double *out_3424776059862686487);
void live_H_14(double *state, double *unused, double *out_6643421425126638860);
void live_h_33(double *state, double *unused, double *out_1599790737135620753);
void live_H_33(double *state, double *unused, double *out_367392009744764525);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}