//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 21/11/2017
//---------------------------

#include "mbs_matrix.h"
#include "math.h"

void mbs_car_kine_wheel(double Pw[4],double Rw[4][4],
                    double Vw[4],double OMw[4],double rnom,
                    double *pen_p,double *rz_p, double *angslip_p, double *angcamb_p,
                    double *slip_p, double Pcontact[4], double Vcontact[4], double Rt_ground[4][4], double dxF[4])
{
    // Rotation matrices
    double **R_Y_I, Rt_Y_I[4][4], R_R_I[4][4], **Rt_R_I, R_T_I[4][4], Rt_T_I[4][4];
    double e1[4], e2[4], e2P[4], e3[4]; // all expressed in I-frame
    double n_I[4] = {0,0,0,1}; // ground normal expressed in I-frame

    // physical quantities
    double pen;
    double sinf, tg_angslip;
    double angslip, angcamb, slip;

    // positions and distances
    double dGQ; 
    double *pOG_I;
    double pOP_I[4];
    double pGQ_T[4], pGQ_I[4];
    double *pOQ_I;

    // velocities and angular velocities
    double *vOG_I, vOG_R[4];
    double *vOQmat_I, vOQmat_R[4], vOQgeo_I[4], vOQgeo_R[4];
    double *omYI_I,omYI_Y[4];
    double omYI_x_pGQ_I[4];
    double omXI_Y[4], omXI_I[4];
    double omXI_x_pGQ_I[4];
   
    // Matching input/output variables and working variables
    pOG_I = Pw;  
    R_Y_I = Rw; R_Y_I[0] = Rw[0]; R_Y_I[1] = Rw[1];  R_Y_I[2] = Rw[2]; R_Y_I[3] = Rw[3];
    vOG_I = Vw;
    omYI_I = OMw;
    pOQ_I =Pcontact;
    vOQmat_I=Vcontact;
    Rt_R_I=Rt_ground;   Rt_R_I[0] = Rt_ground[0]; Rt_R_I[1] = Rt_ground[1]; Rt_R_I[2] = Rt_ground[2]; Rt_R_I[3] = Rt_ground[3];

    /*
    Scheme of the different points (IDEALLY SHOULD BE CHANGED SO THAT IT FITS THE BOOK POINT LABELLING)

                            /
                           /
                          /
                         /
                        . G  (wheel center)
                       /
                      /
                     /
                    /
   ..............  .  ................................  .  .........................
                   Q (contact point )                   O (origin point)
    
    Model Assumptions
    - local flat and horizontal ground
    Conventions
    - wheel spin axis has to be a 2-axis (i.e. associated to a R2 rotation)
    - normal to the ground is point upward (because of z definition)
    Frame Definitions:
    - [I] : inertial frame
    - [Y] : wheel frame                                       (material wheel, Y in book)    
    - [T] : wheel contact point tangential frame                          
    - [R] : wheel/ground contact frame  aligned with the wheel along the longitudinal direction 

    With flat and horizontal ground, [R] has its third axis always point upward. 

    Rotation matrices
    [Y] = R_Y_I*[I]
    ...
    */
    transpose(R_Y_I, Rt_Y_I);

    /*	Frame Definition */

    //e2 = wheel spin axis in [I]
    e2[1] = R_Y_I[2][1];
    e2[2] = R_Y_I[2][2];
    e2[3] = R_Y_I[2][3];
    
    //e2P = wheel spin axis projected in a horizontal plane (expressed in [I])
    e2P[1] = R_Y_I[2][1];
    e2P[2] = R_Y_I[2][2];
    e2P[3] = 0;
    normalize(e2P, e2P);

    //e1 = tangent vector in the wheel median plane (expressed in [I])
    cross_product(e2P, n_I, e1);

    //[R]=R_R_I*[I]
    R_R_I[1][1] = e1[1];
    R_R_I[1][2] = e1[2];
    R_R_I[1][3] = e1[3];

    R_R_I[2][1] = e2P[1];
    R_R_I[2][2] = e2P[2];
    R_R_I[2][3] = e2P[3];

    R_R_I[3][1] = 0;
    R_R_I[3][2] = 0;
    R_R_I[3][3] = 1;

    transpose(R_R_I, Rt_R_I);
    //e3 = vector pointing from the ground toward the wheel center (expressed in [I])
    cross_product(e1, e2, e3);

    //[R_T]=R_T_I*[I]
    R_T_I[1][1] = e1[1];
    R_T_I[1][2] = e1[2];
    R_T_I[1][3] = e1[3];

    R_T_I[2][1] = e2[1];
    R_T_I[2][2] = e2[2];
    R_T_I[2][3] = e2[3];

    R_T_I[3][1] = e3[1];
    R_T_I[3][2] = e3[2];
    R_T_I[3][3] = e3[3];

    transpose(R_T_I, Rt_T_I);

    /*	Camber Angle (f) */
    //	def: angle formed by the wheel median plane with respect to the normal to the ground.
    sinf = R_Y_I[2][3];       	//! valid on horizontal ground !
    angcamb = asin(sinf);
    *angcamb_p = angcamb;

    /*	Contact Point Kinematics */

    //--Position--
    // ground profile under the wheel center
    pOP_I[1] = pOP_I[1];
    pOP_I[2] = pOP_I[2];
    pOP_I[3] = 0.0; // user_GroundLevel(pOP_I[1],pOP_I[2]);
    // distance between the wheel center and the pierce point in the ground.
    dGQ = (pOG_I[3]- pOP_I[3])/cos(angcamb);
    *rz_p = dGQ; // = dGP
    // GQ vector expressed in frame [T]
    pGQ_T[1] = 0;
    pGQ_T[2] = 0;
    pGQ_T[3] = -dGQ;
    // GQ vector expressed in frame [I]
    matrix_product(Rt_T_I, pGQ_T, pGQ_I);
    // absolute position of point Q (expressed in [I])
    vector_sum(pOG_I, pGQ_I, pOQ_I);

    // vertical deflection calculation
    pen =  cos(angcamb)*(rnom - dGQ);
    *pen_p = pen;
    // Vector of force application (from the wheel center, G to the contact point, Q) expressed in [Y]. 
    matrix_product(R_Y_I, pGQ_I, dxF);
    
    //--Speed--
    // "entrainement" velocity of the material contact point (expressed in [I])
    cross_product(omYI_I, pGQ_I, omYI_x_pGQ_I);
    // (absolute) velocity of the material contact point (expressed in [I])
    vector_sum(vOG_I, omYI_x_pGQ_I , vOQmat_I);
    // (absolute) velocity of the material contact point (expressed in [R])
    matrix_product(R_R_I, vOQmat_I, vOQmat_R);

    /* lateral slip angle (angliss) */
    //  def: angle of the geometric point velocity with respect to the wheel median plane.  (computed in R frame).
    // wheel center velocity expressed in [R]
    matrix_product(R_R_I, vOG_I, vOG_R);
    //frozen wheel angular velocity (expressed in [Y])
    matrix_product(R_Y_I, omYI_I, omYI_Y);
    omXI_Y[1] = omYI_Y[1];
    omXI_Y[2] = 0;
    omXI_Y[3] = omYI_Y[3];
    matrix_product(Rt_Y_I, omXI_Y, omXI_I);
    // "entrainement" velocity of geometrical contact point (expressed in [I])
    cross_product(omXI_I, pGQ_I, omXI_x_pGQ_I);
    // velocity of geometrical contact point (expressed in [I])
    vector_sum(vOG_I, omXI_x_pGQ_I, vOQgeo_I);
    // velocity of geometrical contact point (expressed in [Rsol])
    matrix_product(R_R_I, vOQgeo_I, vOQgeo_R);
    // angle computation
    if ((vOQgeo_R[1] >= 1e-3) || (vOQgeo_R[1] <= -1e-3))
    {
        tg_angslip = vOQgeo_R[2] / vOQgeo_R[1];
    }
    else
    {
        tg_angslip = 0;
		//printf("Rentre dans tg_angslip = 0\n");
    }
    angslip = atan(tg_angslip);
    *angslip_p = angslip;

    /* Longitudinal slip (gliss) */
    //  def: ratio between the material contact point long. speed and the wheel center long. speed (expressed in frame R)
    // longitudinal speed of the material contact point : vOQmat_R(1)
    // longitudinal speed of the wheel center           : vOG_R[1]
    if ((vOG_R[1] >= 1e-3) | (vOG_R[1] <= -1e-3))
    {
        slip = -vOQmat_R[1] / vOG_R[1];
    }
    else
    {
        slip = 0;
    }
    *slip_p = slip;

}