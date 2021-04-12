#ifndef MBS_CAR_TIREGROUND_h
#define MBS_CAR_TIREGROUND_h



/**
* \brief compute car wheel kinematics which are necessary inputs for a tire/ground contact model.
*        Note that the car tire is defined by
*           - a nominal radius (from wheel center to contact patch)
*
* \p pOG_I, Pw  [in], Position of the wheel center in the inertial frame.
* \p R_Y_I, Rw  [in], Rotation Matrix of the wheel body.
* \p vOG_I, Vw  [in], Velocity of the wheel center in the inertial frame.
* \p omYI_I,OMw [in], Angular velocity of the wheel center in the inertial frame.
* \p rnom       [in], wheel nominal radius 
* \p pen_p      [out], tire vertical deflection.
* \p rz_p       [out], distance between the wheel center and the pierce point in the ground.
* \p angslip_p  [out], pointer for lateral slip angle.
* \p angcamb_p  [out], pointer for camber angle.
* \p slip_p     [out], pointer for longitudinal slip.
* \p pOQ_I,         [out], Position of the point in contact expressed in the inertial frame.
* \p vOQmat_I, Vct  [out], Velocity of the material point in contact expressed in the inertial frame.
* \p Rt_R_I         [out], transpose of Rotation matrix from [I] to [R].
* \p dxF        [out], Vector of force application (from the wheel center to the contact point) expressed in the rotating wheel frame [Y]
*/
void mbs_car_kine_wheel(double Pw[4], double Rw[4][4],
    double Vw[4], double OMw[4], double rnom,
    double *pen_p, double *rz_p, double *angslip_p, double *angcamb_p,
    double *slip_p, double Pcontact[4], double Vcontact[4], double Rt_ground[4][4], double dxF[4]);


void mbs_car_bakker(double Fwhl[4], double Mwhl[4], double angslip, double angcamb, double slip);

#endif