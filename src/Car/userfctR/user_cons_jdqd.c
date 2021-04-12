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
#include "user_model.h"


void user_cons_jdqd(double *jdqd, MbsData *mbs_data, double tsim)
{
	
	double beta = mbs_data->user_model->Contrainte.beta * M_PI / 180; //30 degres
 
	jdqd[1] = 0;

    double R_p;
	double p_1 = 0.8278; //value found with Matlab (ResultSteerWheel.m : section Curve Fitting GOOD)
	double p_2 = -2.0483;
	double p_3 = -5.1737;
	double p_4 = -1.2708;
	double p_5 = 198.0082;
	double p_low = 159.3;
	if (mbs_data->q[R3_pinion_id] >= -2.951 && mbs_data->q[R3_pinion_id] <= 0)
	{
		jdqd[2] = 2*(mbs_data->qd[T2_rack_id]*(mbs_data->qd[R3_pinion_id])*(4*p_1*pow(-mbs_data->q[R3_pinion_id],3)+3*p_2*pow(-mbs_data->q[R3_pinion_id],2) + 2*p_3*(-mbs_data->q[R3_pinion_id]) + p_4))+((pow(-mbs_data->qd[R3_pinion_id],2)*(-mbs_data->q[T2_rack_id])*(12*p_1*pow(-mbs_data->q[R3_pinion_id],2)+6*p_2*(-mbs_data->q[R3_pinion_id])+2*p_3)));
	
	}
	else if (mbs_data->q[R3_pinion_id] > 0 && mbs_data->q[R3_pinion_id] <= 2.951)
	{
		jdqd[2] = 2*(mbs_data->qd[T2_rack_id]*(-mbs_data->qd[R3_pinion_id])*(4*p_1*pow(mbs_data->q[R3_pinion_id],3)+3*p_2*pow(mbs_data->q[R3_pinion_id],2) + 2*p_3*(mbs_data->q[R3_pinion_id]) + p_4))+((pow(mbs_data->qd[R3_pinion_id],2)*(-mbs_data->q[T2_rack_id])*(12*p_1*pow(mbs_data->q[R3_pinion_id],2)+6*p_2*mbs_data->q[R3_pinion_id]+2*p_3)));
	}
	else
	{
		jdqd[2] = 0;
	}

	jdqd[3] = -(pow(mbs_data->qd[R1_steerwheel_id],2)*2*cos(mbs_data->q[R1_steerwheel_id])*sin(mbs_data->q[R1_steerwheel_id])*cos(beta)*(pow(cos(beta),2)-1))/(pow((pow(cos(beta)*cos(mbs_data->q[R1_steerwheel_id]), 2) + pow(sin(mbs_data->q[R1_steerwheel_id]), 2)),2));
	jdqd[4] = 0;
	jdqd[5] = -(pow(mbs_data->qd[Inter_out_id], 2) * 2 * cos(mbs_data->q[Inter_out_id])*sin(mbs_data->q[Inter_out_id])*cos(beta)*(pow(cos(beta), 2) - 1)) / (pow((pow(cos(beta)*cos(mbs_data->q[Inter_out_id]), 2) + pow(sin(mbs_data->q[Inter_out_id]), 2)), 2));
	jdqd[6] = 0;
   
}
