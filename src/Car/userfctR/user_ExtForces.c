//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//
//
//---------------------------

#include "math.h"

#include "mbs_data.h"
#include "mbs_project_interface.h"

#include "user_all_id.h"
#include "user_model.h"
#include "mbs_car_tireGround.h"


double angle_vecteurs(int droite, MbsData *mbs_data)
{
    // definition
    double R11C, R12C, R11D, R12D, R11G, R12G;
    double norm_u;
    double norm_v;
    double cross_uv[4] = { 0.0,0.0,0.0,0.0 }, dot_uv;
    double angle;
    double sin_A, cos_A;


    R11C = mbs_data->user_model->Rotations.R11C;
    R12C = mbs_data->user_model->Rotations.R12C;

    R11D = mbs_data->user_model->Rotations.R11D;
    R12D = mbs_data->user_model->Rotations.R12D;

    R11G = mbs_data->user_model->Rotations.R11G;
    R12G = mbs_data->user_model->Rotations.R12G;

    double u[4] = { 0.0, R11C, R12C, 0.0 };
    double v[4] = { 0.0, 0.0, 0.0, 0.0 };


    if (droite == 1) //droite
    {
        v[1] = R11D;
        v[2] = R12D;
    }
    else if (droite == 0) //gauche
    {
        v[1] = R11G;
        v[2] = R12G;
    }
    else if (droite == 2) //somme
    {
        v[1] = (R11D + R11G) / 2;
        v[2] = (R12D + R12G) / 2;
    }

    norm_u = norm_dvec_1(u, 3);
    norm_v = norm_dvec_1(v, 3);

    cross_product(u, v, cross_uv);
    dot_uv = scalar_product(u, v);

    cos_A = dot_uv / (norm_u*norm_v);

    if (cross_uv[3] >= 0)
    {
        sin_A = norm_dvec_1(cross_uv, 3) / (norm_u*norm_v);
    }
    else
    {
        sin_A = -norm_dvec_1(cross_uv, 3) / (norm_u*norm_v);
    }

    angle = atan2(sin_A, cos_A);

    return angle;
}



double* user_ExtForces(double PxF[4], double RxF[4][4], 
                       double VxF[4], double OMxF[4], 
                       double AxF[4], double OMPxF[4], 
                       MbsData *mbs_data, double tsim,int ixF)
{
    double Fx=0.0, Fy=0.0, Fz=0.0;
    double Mx=0.0, My=0.0, Mz=0.0;
    double dxF[4] ={0.0, 0.0, 0.0, 0.0};

    // wheel kinematics variables
    double pen, rz, angslip, angcamb, slip, Pcon[4], Vcon[4], Rt_ground[4][4], Rt_ground_t[4][4];
    // tire/ground contact forces variables
    double Fwhl_R[4], Mwhl_R[4], Fwhl_I[4], Mwhl_I[4];

    double *SWr = mbs_data->SWr[ixF];
    // default application point of the force: anchor point to which it is attached
    int idpt = 0;
    idpt = mbs_data->xfidpt[ixF];
    dxF[1] = mbs_data->dpt[1][idpt];
    dxF[2] = mbs_data->dpt[2][idpt];
    dxF[3] = mbs_data->dpt[3][idpt];


double coeff_gliss = 1;
if(mbs_data->tsim > mbs_data->user_model->expe.t_rand && mbs_data->tsim < mbs_data->user_model->expe.t_rand+2 && mbs_data->user_model->expe.t_rand != 0)
{
  //printf("It slips ! \n");
  coeff_gliss = 0.5;
}

/* Begin of user declaration */
    switch (ixF)
    {
        /* Begin of user code */
    case F_av_g_1_id:

		mbs_car_kine_wheel(PxF, RxF, VxF, OMxF, mbs_data->user_model->FrontTire.R, &pen, &rz, &angslip, &angcamb, &slip, Pcon, Vcon, Rt_ground, dxF);
		if (mbs_data->process == 1)// equil static 
		{
			Fwhl_R[1] = 0.0;
			Fwhl_R[2] = 0.0;
			Fwhl_R[3] = mbs_data->user_model->FrontTire.K*pen;
			Mwhl_R[1] = 0.0;
			Mwhl_R[2] = 0.0;
			Mwhl_R[3] = 0.0;
		}
		else if (mbs_data->process == 2) // dynamic
		{
			if (pen > 0)
			{
				
					Fwhl_R[3] = mbs_data->user_model->FrontTire.K*pen;

				mbs_car_bakker(Fwhl_R, Mwhl_R, angslip, angcamb, slip);
				//mbs_car_bakker_lin_0(Fwhl_R, Mwhl_R, angslip, angcamb, slip);

                               Fwhl_R[1] *= coeff_gliss;
			       Fwhl_R[2] *= coeff_gliss;
                               Mwhl_R[3] *= coeff_gliss;
			}
			else
			{
				Fwhl_R[1] = 0.0;
				Fwhl_R[2] = 0.0;
				Fwhl_R[3] = 0.0;
				Mwhl_R[3] = 0.0;
			}
		}

		matrix_product(Rt_ground, Fwhl_R, Fwhl_I);
		matrix_product(Rt_ground, Mwhl_R, Mwhl_I);

		break;
		/* End of user code */
    case F_av_d_1_id:
    case F_ar_g_1_id:
    case F_ar_d_1_id:

        mbs_car_kine_wheel(PxF, RxF, VxF, OMxF, mbs_data->user_model->FrontTire.R, &pen, &rz, &angslip, &angcamb, &slip, Pcon, Vcon, Rt_ground, dxF);
        if (mbs_data->process == 1)// equil static 
        {
            Fwhl_R[1] = 0.0;
            Fwhl_R[2] = 0.0;
            Fwhl_R[3] = mbs_data->user_model->FrontTire.K*pen;
            Mwhl_R[1] = 0.0;
            Mwhl_R[2] = 0.0;
            Mwhl_R[3] = 0.0;
        }
        else if (mbs_data->process == 2) // dynamic
        {
			if (pen > 0)
			{
				
				Fwhl_R[3] = mbs_data->user_model->FrontTire.K*pen;
			    

                mbs_car_bakker(Fwhl_R, Mwhl_R, angslip, angcamb, slip);
                //mbs_car_bakker_lin_0(Fwhl_R, Mwhl_R, angslip, angcamb, slip);


                     Fwhl_R[1] *= coeff_gliss;
	             Fwhl_R[2] *= coeff_gliss;
                     Mwhl_R[3] *= coeff_gliss;
            }
            else
            {
                Fwhl_R[1] = 0.0;
                Fwhl_R[2] = 0.0;
                Fwhl_R[3] = 0.0;
                Mwhl_R[3] = 0.0;
            }
        }

        matrix_product(Rt_ground, Fwhl_R, Fwhl_I);
        matrix_product(Rt_ground, Mwhl_R, Mwhl_I);

        break;
        /* End of user code */
    }

   switch (ixF)
    {
        
    case F_av_g_1_id:
       set_output(Fwhl_I[2],"F_lat_av_g");
     break;

     case F_ar_g_1_id:
       set_output(Fwhl_I[2],"F_lat_ar_g");
     break;

      case F_av_d_1_id:
       set_output(Fwhl_I[2],"F_lat_av_d");
     break;


      case F_ar_d_1_id:
       set_output(Fwhl_I[2],"F_lat_ar_d");
     break;


    }


    SWr[1] = Fwhl_I[1];
    SWr[2] = Fwhl_I[2];
    SWr[3] = Fwhl_I[3];
    SWr[4] = Mwhl_I[1];
    SWr[5] = Mwhl_I[2];
    SWr[6] = Mwhl_I[3];
    SWr[7]=dxF[1];
    SWr[8]=dxF[2];
    SWr[9]=dxF[3];


 

    // Extra Kinematic Computation
    if (1)
    {
        transpose(Rt_ground, Rt_ground_t);
        switch (ixF)
        {
        case F_av_g_1_id:
            mbs_data->user_model->Rotations.R11G = Rt_ground_t[1][1];
            mbs_data->user_model->Rotations.R12G = Rt_ground_t[1][2];
            break;
        case F_av_d_1_id:
            mbs_data->user_model->Rotations.R11D = Rt_ground_t[1][1];
            mbs_data->user_model->Rotations.R12D = Rt_ground_t[1][2];
            break;
        }


        double angle;
        MbsSensor psens[1];                        // Creation of a pointer to a sensor struct.
        allocate_sensor(psens, mbs_data->njoint);  // Allocate the Jacobian at the correct dimension
        init_sensor(psens, mbs_data->njoint);      // Initialize all value to zero

        mbs_sensor(psens, mbs_data, Sensor_chassis_id); // Compute the sensor

        mbs_data->user_model->Rotations.R11C = psens->R[1][1];
        mbs_data->user_model->Rotations.R12C = psens->R[1][2];

        switch (ixF)
        {
        case F_av_g_1_id:
            angle = angle_vecteurs(0, mbs_data);
            mbs_data->user_model->SteeringAssembly.delta_left = angle;
            break;
        case F_av_d_1_id:
            angle = angle_vecteurs(1, mbs_data);
            mbs_data->user_model->SteeringAssembly.delta_right = angle;
            break;
        }

        free_sensor(psens);                        // Free the memory (always better)
    }


    return SWr;
}

 
