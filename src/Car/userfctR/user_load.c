/** ---------------------------
  * Robotran - MBsysC
  * 
  * Template file for equilibrium module
  * 
  * This files enable the user to call custom code just 
  * after loading the project. 
  * The call is done inside the mbs_load function.
  * 
  * (c) Universite catholique de Louvain
  *     
  */

#include "math.h"

#include "mbs_data.h"

/*! \brief user own initialization functions
 *
 * \param[in,out] mbs_data data structure of the model
 *
 */
void user_load_post(MbsData *mbs_data)
{
    
    /*
        Do not edit this file ! This is the example file in mbsysC workspace
        Instead, copy/paste this file in your userfctR/folder of your project
        
        This function is called at the end of mbs_load() and is usefull when compiling the project 
        separately in order for example to use Simulink (when the executable does not call the main.c)
    */
}
