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
#include "user_model.h"
#include "set_output.h"
#include "useful_functions.h"

#include "user_all_id.h"
#include "user_model.h"



double* user_JointForces(MbsData *mbs_data, double tsim)
{
/*-- Begin of user code --*/

    if (tsim == 0.0) // equilibrium process and modal analysis
    {
        mbs_data->Qq[R2_wheel_rr_lt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion;
        mbs_data->Qq[R2_wheel_rr_rt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion;
        mbs_data->Qq[T2_rack_id] = mbs_data->user_model->EquilQuantities.Qrack;
    }

    if(mbs_data->process == 2)
    {
    	/********/
    	/* TODO */
    	/********/

    	// fix le trigger
    	// 		'calibration' ? (valeur init pas cst)
    	//		montage ressort : pas bon rappel, trop aléatoire
    	//
    	// valeurs de couple
    	//		freinage : normalement ok pour un trigger à 400, suffit de recalculer avec Excel
    	// 		acceleration : calculer le bon couple à mettre aux roues pour avoir du 4-5-6 m/s²
    	//					   puis calcul Excel comme pour frein
    	//
    	// tester les différentes lois

    	/********/

        int pedals_mode = mbs_data->user_model->pedals.mode;

        // PEDALS ON : Acceleration and braking
        if(pedals_mode == 1) {
        	double speed = 3.6 * sqrt(pow(mbs_data->qd[T1_chassis_id],2)+pow(mbs_data->qd[T2_chassis_id],2));
            double accel = sqrt(pow(mbs_data->qdd[T1_chassis_id],2)+pow(mbs_data->qdd[T2_chassis_id],2));

            double ped1 = mbs_data->user_model->pedals.ped1;
            double ped2 = mbs_data->user_model->pedals.ped2;
            double ped1_rest = 7000, ped1_max = 22000;
            double ped2_rest = 1000, ped2_max = 14000;
            // on prend max. 5m/s², traction 100%
            // --> simus essai / erreur pour trouver le couple correspondant
            double accel_ft_max = 1800;
            // cf. TFE Theo
            // ratio traction / propulsion : 65% / 35%
            double brake_ft_max = -1235;
            double brake_rr_max = -665;        

            int accel_mode = 1; //1: cst, 2: lin, 3: exp, 4: log
            int brake_mode = 1; //1: cst, 2: lin, 3: exp, 4: log
           
            // Pedale de frein enfoncee & vitesse > 10 km/h
            if(ped1 < ped1_rest && ped2 >= ped2_rest && speed > 10) {
                
            	if(brake_mode == 1) {
            		mbs_data->Qq[R2_wheel_rr_lt_id] = brake_rr_max;
	                mbs_data->Qq[R2_wheel_rr_rt_id] = brake_rr_max;
	                mbs_data->Qq[R2_wheel_ft_lt_id] = brake_ft_max;
	                mbs_data->Qq[R2_wheel_ft_rt_id] = brake_ft_max;
            	}
            	else if(brake_mode == 2) {
            		mbs_data->Qq[R2_wheel_rr_lt_id] = brake_rr_max / (ped2_max-ped2_rest) * (ped2-ped2_rest);
             	    mbs_data->Qq[R2_wheel_rr_rt_id] = brake_rr_max / (ped2_max-ped2_rest) * (ped2-ped2_rest);
               		mbs_data->Qq[R2_wheel_ft_lt_id] = brake_ft_max / (ped2_max-ped2_rest) * (ped2-ped2_rest);
                	mbs_data->Qq[R2_wheel_ft_rt_id] = brake_ft_max / (ped2_max-ped2_rest) * (ped2-ped2_rest);
            	}
            	else if(brake_mode == 3) {
            		mbs_data->Qq[R2_wheel_rr_lt_id] = - 0.826 * exp(0.0005 * ped2);
             	    mbs_data->Qq[R2_wheel_rr_rt_id] = - 0.826 * exp(0.0005 * ped2);
               		mbs_data->Qq[R2_wheel_ft_lt_id] = - 0.811 * exp(0.0005 * ped2);
                	mbs_data->Qq[R2_wheel_ft_rt_id] = - 0.811 * exp(0.0005 * ped2);
            	}
            	else if(brake_mode == 4) {
            		mbs_data->Qq[R2_wheel_rr_lt_id] = - 187 * log(ped2) + 1120.7;
             	    mbs_data->Qq[R2_wheel_rr_rt_id] = - 187 * log(ped2) + 1120.7;
               		mbs_data->Qq[R2_wheel_ft_lt_id] = - 347.4 * log(ped2) + 2081.2;
                	mbs_data->Qq[R2_wheel_ft_rt_id] = - 347.4 * log(ped2) + 2081.2;
            	}
                
            }
            // Pedale d'acceleration enfoncee & vitesse < 120 km/h
            else if(ped1 >= ped1_rest && ped2 < ped2_rest && speed < 120) {
                
                if(accel_mode == 1) {
               		mbs_data->Qq[R2_wheel_ft_lt_id] = accel_ft_max;
                	mbs_data->Qq[R2_wheel_ft_rt_id] = accel_ft_max;
                }
                else if(accel_mode == 2) {
               		mbs_data->Qq[R2_wheel_ft_lt_id] = accel_ft_max / (ped1_max-ped1_rest) * (ped1-ped1_rest);
                	mbs_data->Qq[R2_wheel_ft_rt_id] = accel_ft_max / (ped1_max-ped1_rest) * (ped1-ped1_rest);
                }
                else if(accel_mode == 3) {
               		mbs_data->Qq[R2_wheel_ft_lt_id] = 0;
                	mbs_data->Qq[R2_wheel_ft_rt_id] = 0;	
                }
                else if(accel_mode == 4) {
               		mbs_data->Qq[R2_wheel_ft_lt_id] = 0;
                	mbs_data->Qq[R2_wheel_ft_rt_id] = 0;
                }
             
            }
            else {
                // Aucune pedale enfoncee (ou les deux) --> on ne fait rien
            }
        }

        // PEDALS OFF : PID Controller
        else if(pedals_mode == 2) {
            double error = mbs_data->user_model->PID.velocity - (sqrt(pow(mbs_data->qd[1],2)+pow(mbs_data->qd[2],2)));
          
            if(error >= -0.1 && error <= 0.1) mbs_data->user_model->PID.sum_error = 0;
          
            mbs_data->user_model->PID.sum_error += error;
          
            double error_variation = error-mbs_data->user_model->PID.previous_error;

            mbs_data->user_model->PID.previous_error = error;

            double commande  = mbs_data->user_model->PID.Kp*error + mbs_data->user_model->PID.Ki*mbs_data->user_model->PID.sum_error + mbs_data->user_model->PID.Kd*error_variation;

            if(commande > 750)
            {
                commande = 750;
            }

            if(mbs_data->user_model->PID.brake == 1 && sqrt(pow(mbs_data->qd[1],2)+pow(mbs_data->qd[2],2)) > 25/3.6) //brake with space barre
            {
                mbs_data->Qq[R2_wheel_rr_lt_id] = -665*0.4;
                mbs_data->Qq[R2_wheel_rr_rt_id] = -665*0.4;
                mbs_data->Qq[R2_wheel_ft_lt_id] = -1235*0.4;
                mbs_data->Qq[R2_wheel_ft_rt_id] = -1235*0.4;
            }
            else
            {
                mbs_data->Qq[R2_wheel_rr_lt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion + commande;
                mbs_data->Qq[R2_wheel_rr_rt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion + commande;
                mbs_data->Qq[R2_wheel_ft_lt_id] = 0;
                mbs_data->Qq[R2_wheel_ft_rt_id] = 0;
            }
        }

    }

    // ANTI-ROLL BAR
    mbs_data->Qq[R2_def_bar_ft_id] = -mbs_data->user_model->FrontSuspension.C_bar*mbs_data->q[R2_def_bar_ft_id];
    mbs_data->Qq[R2_def_bar_rr_id] = -mbs_data->user_model->RearSuspension.C_bar*mbs_data->q[R2_def_bar_rr_id];


    return mbs_data->Qq;
}
