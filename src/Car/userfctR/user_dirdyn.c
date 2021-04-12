/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for direct dynamics module
 * 
 * This files enable the user to call custom at
 * specific places in the time simulation. It is a template
 * file that can be edited by the user.
 * 
 * (c) Universite catholique de Louvain
 *     
 */

#include "math.h"

#include "mbs_data.h"
#include "mbs_dirdyn_struct.h"
#include "set_output.h"
#include "user_all_id.h"
#include "mbs_sensor.h"


#include "user_model.h"



/*! \brief user own initialization functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_init(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{
   
}

/*! \brief user own loop functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_loop(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{
 
     (*mbs_data->user_model->thread.thread_struct->pointeur_give_torque_access)();
    
   // set_output(mbs_dd->mbs_aux->n_iter_close_loop,"iter_NR");



   //runer code pour chaque sensor

	/*int id_h = Sensor_h_id;

	// retrieve the pointer to the sensor structure defined in mbs_aux
	MbsSensor *PtrSensor = mbs_dd->mbs_aux->psens;

	// compute the sensor (position, velocity...)
	mbs_comp_S_sensor(PtrSensor, mbs_data, id_h);

	// save the vertical acceleration
	set_output(PtrSensor->P[1], "X_position_sensor_h");
	set_output(PtrSensor->P[2], "Y_position_sensor_h");
	set_output(PtrSensor->P[3], "Z_position_sensor_h");*/



	/*int id_l = Sensor_l_id;

	// retrieve the pointer to the sensor structure defined in mbs_aux
	MbsSensor *PtrSensor = mbs_dd->mbs_aux->psens;

	// compute the sensor (position, velocity...)
	mbs_comp_S_sensor(PtrSensor, mbs_data, id_l);

	// save the vertical acceleration
	set_output(PtrSensor->P[1], "X_position_sensor_l");
	set_output(PtrSensor->P[2], "Y_position_sensor_l");
	set_output(PtrSensor->P[3], "Z_position_sensor_l");*/
	


	int id_c = Sensor_c_id;

	// retrieve the pointer to the sensor structure defined in mbs_aux
	MbsSensor *PtrSensor = mbs_dd->mbs_aux->psens;

	// compute the sensor (position, velocity...)
	mbs_comp_S_sensor(PtrSensor, mbs_data, id_c);

	// save the vertical acceleration
	//set_output(PtrSensor->P[1], "X_position_sensor_c");
	//set_output(PtrSensor->P[2], "Y_position_sensor_c");
	//set_output(PtrSensor->P[3], "Z_position_sensor_c");

}

/*! \brief user own finishing functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_finish(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{

}


