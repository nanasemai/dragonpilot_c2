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
void live_H(double *in_vec, double *out_5722601585300423481);
void live_err_fun(double *nom_x, double *delta_x, double *out_7437757691352687044);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2356113789153817818);
void live_H_mod_fun(double *state, double *out_2027243642553433340);
void live_f_fun(double *state, double dt, double *out_8471030786793314343);
void live_F_fun(double *state, double dt, double *out_92358162625652856);
void live_h_4(double *state, double *unused, double *out_3796505834477688763);
void live_H_4(double *state, double *unused, double *out_3319294905611910086);
void live_h_9(double *state, double *unused, double *out_8006315549488211489);
void live_H_9(double *state, double *unused, double *out_6208156457891989428);
void live_h_10(double *state, double *unused, double *out_3133804252011675785);
void live_H_10(double *state, double *unused, double *out_2249826230239014576);
void live_h_12(double *state, double *unused, double *out_76440570906724364);
void live_H_12(double *state, double *unused, double *out_3940393930659503753);
void live_h_35(double *state, double *unused, double *out_4770267417306239845);
void live_H_35(double *state, double *unused, double *out_6685956962984517462);
void live_h_32(double *state, double *unused, double *out_4811611708318517052);
void live_H_32(double *state, double *unused, double *out_1931099657671759736);
void live_h_13(double *state, double *unused, double *out_3067489702502385606);
void live_H_13(double *state, double *unused, double *out_2423941019219024521);
void live_h_14(double *state, double *unused, double *out_8006315549488211489);
void live_H_14(double *state, double *unused, double *out_6208156457891989428);
void live_h_33(double *state, double *unused, double *out_5959759342794358757);
void live_H_33(double *state, double *unused, double *out_8610230106086176550);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}