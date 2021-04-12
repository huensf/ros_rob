/*! 
 * \author Nicolas Van der Noot
 * \file user_realtime_events.c
 * \brief Handles the events coming from the keyboard or from other sources via the SDL framework
 *
 * In order to use C++ features, you just need to change the extension of this file (.c) to .cc (or .cpp).
 */
 
#ifdef SDL

#include "realtime.h"
#include "events_sdl.h"
#include "user_realtime.h"
#include "user_model.h"
#include "user_all_id.h"

/*! \brief handle inputs comming from the keyboard
 * 
 * \param[in,out] mbs_data Robotran main structure
 * \param[in] realtime real-time structure
 * \param[in] cur_t_usec curent time [us]
 * \param[in] keystates state of the keys (from the keyboard)
 *
 * Use keystates['key code'] (see https://wiki.libsdl.org/SDL_Scancode)
 * to detect when the corresponding key is pressed and add your own functions
 * according to the key pressed.
 *
 * Call 'wait_key(realtime, cur_t_usec, time);' in the statement where you replace 'time'
 * by a time in seconds if you want to wait before detecting a new user command.
 * Pay attention, when you call 'wait_key', the program is automatically kept awake
 * for a few seconds. Consequently, the process can not be realeased after a few seconds
 * during a break in case 'wait_key' is always called.
 *
 * example:
 *      if (keystates[SDL_SCANCODE_UP]) 
 *      {
 *          mbs_data->user_IO->my_command++;
 *          wait_key(realtime, cur_t_usec, 0.1);
 *      }
 *      else if (keystates[SDL_SCANCODE_DOWN]) 
 *      {
 *          mbs_data->user_IO->my_command--;
 *          wait_key(realtime, cur_t_usec, 0.1);
 *      }
 */
void user_keyboard(MbsData* mbs_data, Simu_realtime *realtime, int cur_t_usec, const Uint8 *keystates)
{
     /* if (keystates[SDL_SCANCODE_RIGHT] && mbs_data->q[R1_steerwheel_id] < 9 && mbs_data->q[R1_steerwheel_id] > -9) 
       {
            mbs_data->q[R1_steerwheel_id] =  mbs_data->q[R1_steerwheel_id]+0.008;
           wait_key(realtime, cur_t_usec, 0.001);
       }
       else if (keystates[SDL_SCANCODE_LEFT] && mbs_data->q[R1_steerwheel_id] < 9 && mbs_data->q[R1_steerwheel_id] > -9) 
       {
            mbs_data->q[R1_steerwheel_id] =  mbs_data->q[R1_steerwheel_id] -0.008;
           wait_key(realtime, cur_t_usec, 0.001);
       }
       else
       {
          mbs_data->q[R1_steerwheel_id]=mbs_data->q[R1_steerwheel_id]*0.995;
          wait_key(realtime, cur_t_usec, 0.001);
       }*/

    
      if (keystates[SDL_SCANCODE_SPACE] )
       {
            mbs_data->user_model->PID.brake = 1;
            wait_key(realtime, cur_t_usec, 0.001);
       }
       else
       {
          mbs_data->user_model->PID.brake = 0;
          wait_key(realtime, cur_t_usec, 0.001);
       }
       
}

/*! \brief handle inputs comming from joysticks axes
 * 
 * \param[in,out] mbs_data Robotran main structure
 * \param[in] realtime real-time structure
 * \param[in] nb_joysticks number of joysticks detected
 *
 * Use get_Joystick_axis(int joystickID, int axisID, Simu_realtime *realtime)
 * to return the value associated with the joystick number joystickID
 *
 * joystickID: ID of the joystick (starting at 0)
 * axisID: ID of the axis (starting at 0)
 *
 * get_Joystick_axis returns a value in [-1 ; 1] or -10 if this joystickID is not available
 *
 * plugged joysticks are automatically detected at launch (see nb_joysticks)
 *
 * example:
 *     joystick_val = get_Joystick_axis(0, 0, realtime);
 *     mbs_data->user_IO->my_variable = joystick_val * scaling_factor;
 */
void user_joystick_axes(MbsData* mbs_data, Simu_realtime *realtime, int nb_joysticks)
{

}

/*! \brief handle inputs comming from joysticks buttons
 * 
 * \param[in,out] mbs_data Robotran main structure
 * \param[in] buttonID ID of the joystick button
 *
 * buttonID takes a value (usually starting at 0) when a joystick button is pressed
 *
 * Use printf to detect which 'buttonID' is used for the different buttons.
 *
 * example:
 *      if (buttonID == 2) 
 *      {
 *          mbs_data->user_IO->my_command++;
 *      }
 */
void user_joystick_buttons(MbsData* mbs_data, int buttonID)
{

}

#endif
