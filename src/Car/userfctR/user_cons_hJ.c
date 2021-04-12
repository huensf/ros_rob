//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"

#include "mbs_data.h"
#include "user_all_id.h"
#include "useful_functions.h"
#include "user_model.h"


void user_cons_hJ(double *h, double **Jac, MbsData *mbs_data, double tsim)
{
	//NEED to be set in user_cons_jdqd.c too 
	double R_p;
	double p_1 = 0.8278; //value found with Matlab (ResultSteerWheel.m : section Curve Fitting GOOD)
	double p_2 = -2.0483;
	double p_3 = -5.1737;
	double p_4 = -1.2708;
	double p_5 = 198.0082;
	double p_low = 159.3;
	if (mbs_data->q[R3_pinion_id] >= -2.951 && mbs_data->q[R3_pinion_id] <= 0)
	{
		R_p = p_1*pow(-mbs_data->q[R3_pinion_id],4) + p_2*pow(-mbs_data->q[R3_pinion_id],3) + p_3*pow(-mbs_data->q[R3_pinion_id],2) + p_4*(-mbs_data->q[R3_pinion_id])  + p_5; 

		Jac[2][R3_pinion_id] = 1 - mbs_data->q[T2_rack_id] *(-1)*(4*p_1*pow(-mbs_data->q[R3_pinion_id],3) + 3*p_2*pow(-mbs_data->q[R3_pinion_id],2) + 2*p_3*(-mbs_data->q[R3_pinion_id]) + p_4 );
	}
	else if (mbs_data->q[R3_pinion_id] > 0 && mbs_data->q[R3_pinion_id] <= 2.951)
	{
		R_p = p_1*pow(mbs_data->q[R3_pinion_id],4) + p_2*pow(mbs_data->q[R3_pinion_id],3) + p_3*pow(mbs_data->q[R3_pinion_id],2) + p_4*(mbs_data->q[R3_pinion_id])  + p_5; 

		Jac[2][R3_pinion_id] = 1 - mbs_data->q[T2_rack_id] * (4*p_1*pow(mbs_data->q[R3_pinion_id],3) + 3*p_2*pow(mbs_data->q[R3_pinion_id],2) + 2*p_3*(mbs_data->q[R3_pinion_id]) + p_4);
	}
	else
	{
		R_p = p_low;

		Jac[2][R3_pinion_id] = 1;
	}
	
	//NEED to be set in user_cons_jdqd.c too 
	double beta = mbs_data->user_model->Contrainte.beta *M_PI/180;

	h[1] = mbs_data->q[T2_rack_id] + mbs_data->q[T2_rackp_id];
	h[2] = mbs_data->q[R3_pinion_id] - R_p * mbs_data->q[T2_rack_id];
	if (mbs_data->q[R1_steerwheel_id] > M_PI)
	{
		h[3] =  mbs_data->q[Inter_in_id] - atan2(sin(mbs_data->q[R1_steerwheel_id]), cos(beta)*cos(mbs_data->q[R1_steerwheel_id])) - 2 * M_PI;
	}
	else if (mbs_data->q[R1_steerwheel_id] <= -M_PI)
	{
		h[3] = mbs_data->q[Inter_in_id] - atan2(sin(mbs_data->q[R1_steerwheel_id]), cos(beta)*cos(mbs_data->q[R1_steerwheel_id])) + 2 * M_PI;
	}
	else
	{
		h[3] =  mbs_data->q[Inter_in_id] - atan2(sin(mbs_data->q[R1_steerwheel_id]), cos(beta)*cos(mbs_data->q[R1_steerwheel_id]));
	}
	h[4] = mbs_data->q[Inter_in_id] + M_PI/2 - mbs_data->q[Inter_out_id];
	if (mbs_data->q[Inter_out_id] > 3 * M_PI)
	{
		h[5] = mbs_data->q[Redressement_id] - atan2(sin(mbs_data->q[Inter_out_id]), cos(beta)*cos(mbs_data->q[Inter_out_id])) - 4 * M_PI;
	}
	else if (mbs_data->q[Inter_out_id] > M_PI && mbs_data->q[Inter_out_id] <= 3 * M_PI)
	{
		h[5] = mbs_data->q[Redressement_id] - atan2(sin(mbs_data->q[Inter_out_id]), cos(beta)*cos(mbs_data->q[Inter_out_id])) - 2 * M_PI;
	}
	else if (mbs_data->q[Inter_out_id] <= -M_PI)
	{
		h[5] = mbs_data->q[Redressement_id] - atan2(sin(mbs_data->q[Inter_out_id]), cos(beta)*cos(mbs_data->q[Inter_out_id])) + 2 * M_PI;
	}
	else
	{
		h[5] = mbs_data->q[Redressement_id] - atan2(sin(mbs_data->q[Inter_out_id]), cos(beta)*cos(mbs_data->q[Inter_out_id]));
	}
	h[6] = mbs_data->q[Redressement_id] - M_PI / 2 - mbs_data->q[R3_pinion_id];
	
	Jac[1][T2_rack_id] = 1;
	Jac[1][T2_rackp_id] = 1;
	Jac[2][T2_rack_id] = -R_p; 
	//Jac[2][R3_pinion_id] already compute juste above
	Jac[3][Inter_in_id] = 1;
	Jac[3][R1_steerwheel_id] = -(cos(beta)) / (pow(cos(beta)*cos(mbs_data->q[R1_steerwheel_id]),2)+pow(sin(mbs_data->q[R1_steerwheel_id]),2));
	Jac[4][Inter_in_id] = 1;
	Jac[4][Inter_out_id] = -1;
	Jac[5][Redressement_id] = 1;
	Jac[5][Inter_out_id] = -(cos(beta)) / (pow(cos(beta)*cos(mbs_data->q[Inter_out_id]), 2) + pow(sin(mbs_data->q[Inter_out_id]), 2));
	Jac[6][Redressement_id] = 1;
	Jac[6][R3_pinion_id] = -1;
}
