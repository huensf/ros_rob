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
#include "mbs_set.h"
#include "user_model.h"



void user_DrivenJoints(MbsData *mbs_data,double tsim)
{
        //Update position and velocity of the steerwheel with the data on topic
        if(mbs_data->process ==2) //direct dynamics
         {
            (*mbs_data->user_model->thread.thread_struct->pointeur_give_pos_vit_ped_access)();
         }
      
	/*****************************************
	/*  Graph angle de roue vs Steer ratio   *
	/****************************************/
	/*if (tsim > 0.0 && tsim <= 9.0)
	{
		mbs_data->q[R1_steerwheel_id] = -tsim;
		mbs_data->qd[R1_steerwheel_id] = -1;
		mbs_data->qdd[R1_steerwheel_id] = 0;
	}
       else if (tsim > 9.0 && tsim <= 27.0)
	{
		mbs_data->q[R1_steerwheel_id] = -18+tsim;
		mbs_data->qd[R1_steerwheel_id] = 1;
		mbs_data->qdd[R1_steerwheel_id] = 0;
	}*/

        
	/********************************
	/*  Double changement de bande  *
	/********************************/
	/*
	if (tsim > 2.0 && tsim <= 3.0)
	{
		mbs_data->q[R1_steerwheel_id] = 1*tsim-2.0;
		mbs_data->qd[R1_steerwheel_id] = 1;
		mbs_data->qdd[R1_steerwheel_id] = 0;
	}
	else if (tsim > 3.0 && tsim <=5.0 )
	{
		mbs_data->q[R1_steerwheel_id] = -tsim+4.0;
		mbs_data->qd[R1_steerwheel_id] = -1;
		mbs_data->qdd[R1_steerwheel_id] = 0;
	}
	else if (tsim > 5.0 && tsim <= 6.0)
	{
		mbs_data->q[R1_steerwheel_id] = 0.5*tsim-3.5;
		mbs_data->qd[R1_steerwheel_id] = 0.5;
		mbs_data->qdd[R1_steerwheel_id] = 0;
	}
	else if (tsim > 6.0 && tsim <= 6.5)
	{
		mbs_data->q[R1_steerwheel_id] = -1.4*tsim + 7.9;
		mbs_data->qd[R1_steerwheel_id] = -1.4;
		mbs_data->qdd[R1_steerwheel_id] = 0;
	}
	else if (tsim > 6.5 && tsim <= 8.0)
	{
		mbs_data->q[R1_steerwheel_id] = 1.64*tsim - 11.86;
		mbs_data->qd[R1_steerwheel_id] = 1.64;
		mbs_data->qdd[R1_steerwheel_id] = 0;
	}
	else if (tsim > 8.0 && tsim <= 9.0)
	{
		mbs_data->q[R1_steerwheel_id] = -1.26*tsim + 1.26*9;
		mbs_data->qd[R1_steerwheel_id] = -1.26;
		mbs_data->qdd[R1_steerwheel_id] = 0;
	}
	*/
	/********************************
	/*  Double changement de bande  *
	/********************************/
	/*
	if (tsim > 2.0 && tsim <= 4.0) //more realistic but less precision 
	{
	mbs_data->q[R1_steerwheel_id] = -0.9370* pow(tsim,2) + 5.6218*tsim - 7.4497;
	mbs_data->qd[R1_steerwheel_id] = (2*-0.9370)*tsim + 5.6218;
	mbs_data->qdd[R1_steerwheel_id] = (2 * -0.9370);
	}
	else if (tsim > 4.0 && tsim <= 6.0)
	{
	mbs_data->q[R1_steerwheel_id] = 0.7031*pow(tsim,2)-7.2810*tsim + 17.82;
	mbs_data->qd[R1_steerwheel_id] = 2*0.7031*tsim -7.2810;
	mbs_data->qdd[R1_steerwheel_id] = 2 * 0.7031;
	}
	else if (tsim > 6.0 && tsim <= 7.245)
	{
	mbs_data->q[R1_steerwheel_id] = 2.1166*pow(tsim, 2) - 27.4712*tsim + 88;
	mbs_data->qd[R1_steerwheel_id] = 2 * 2.1166*tsim - 27.4712;
	mbs_data->qdd[R1_steerwheel_id] = 2 * 2.1166;
	}
	else if (tsim > 7.245 && tsim <= 9)
	{
	mbs_data->q[R1_steerwheel_id] = -1.4895*pow(tsim, 2) +24.0854*tsim -96.2850;
	mbs_data->qd[R1_steerwheel_id] = 2 * -1.4895*tsim + 24.0854;
	mbs_data->qdd[R1_steerwheel_id] = 2 * -1.4895;
	}
	else
	{
		mbs_data->q[R1_steerwheel_id] = 0;
		mbs_data->qd[R1_steerwheel_id] = 0;
		mbs_data->qdd[R1_steerwheel_id] = 0;
	}
	*/

        /**************
	/*  Real time *
	/*************/

    /*  if(mbs_data->user_model->command_key.torque < -8 && tsim > 0.0)
      {
          mbs_data->q[R1_steerwheel_id] = -8;
       }
     else if(mbs_data->user_model->command_key.torque >  8 && tsim > 0.0)
      {
          mbs_data->q[R1_steerwheel_id] = 8;
       }
      else if(mbs_data->user_model->command_key.torque <  8 && tsim > 0.0 || mbs_data->user_model->command_key.torque >  -8 && tsim > 0.0)
       {
          mbs_data->q[R1_steerwheel_id] = mbs_data->user_model->command_key.torque;
       }*/

 
}

 
