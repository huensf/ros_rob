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
#include "user_IO.h"

#include "user_all_id.h"
#include "user_model.h"

double user_LinkForces(double Z, double Zd, MbsData *mbs_data, double tsim, int ilnk)
{
/*-- Begin of user code --*/

    double Flink = 0;
    double K, D, L0;

    if (ilnk == Amortisseur_av_g_1_id || ilnk == Amortisseur_av_d_1_id)
    {
        K = mbs_data->user_model->FrontSuspension.K;
        D = mbs_data->user_model->FrontSuspension.D;
        L0 = mbs_data->user_model->FrontSuspension.L0;
        Flink = K*(Z - L0) + D*Zd;
    }
    else if (ilnk == Amortisseur_ar_g_1_id || ilnk == Amortisseur_ar_d_1_id)
    {
        K = mbs_data->user_model->RearSuspension.K;
        D = mbs_data->user_model->RearSuspension.D;
        L0 = mbs_data->user_model->RearSuspension.L0;
        Flink = K*(Z - L0) + D*Zd;
    }

/*-- End of user code --*/    
    
    return  Flink;
}
