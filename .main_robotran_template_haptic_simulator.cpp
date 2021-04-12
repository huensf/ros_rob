   /**
    *
    *   Universite catholique de Louvain
    *   CEREM : Centre for research in mechatronics
    *   http://www.robotran.be  
    *   Contact : info@robotran.be
    *
    *
    *   MBsysC main script template for simple model:
    *   -----------------------------------------------
    *    This template loads the data file *.mbs and execute:
    *      - the coordinate partitioning module
    *      - the direct dynamic module (time integration of
    *        equations of motion).
    *    It may be adapted and completed by the user.
    * 
    *    (c) Universite catholique de Louvain
    *
    * To turn this file as a C++ file, just change its extension to .cc (or .cpp).
    * If you plan to use some C++ files, it is usually better that the main is compiled as a C++ function.
    * Currently, most compilers do not require this, but it is a safer approach to port your code to other computers.
    */

/*********************************************************************************************/
/* Template for the main Robotran in order to use a vehicle in the haptic feedback simulator */
/*********************************************************************************************/
/*
1) For more information please read the tutorial on : 
2) Read all the template and complet it with the parameters and functions of your own vehicle
3) Dont forget to include the other header files you need
*/



#include <stdio.h>
#include "mbs_data.h"
#include "mbs_dirdyn.h"
#include "mbs_part.h"
#include "realtime.h"
#include "mbs_set.h"
#include "mbs_load_xml.h"
#include "cmake_config.h"

#include "mbs_equil.h"
#include "user_all_id.h"
#include "user_model.h"
//#include "thread_struct.h"

// ROS and topic's msg
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_rob/Torque_msg.h"
#include "ros_rob/Pos_vit_msg.h"
#include "std_msgs/UInt8MultiArray.h"

#include <sstream>

#include "pthread.h"
#include <bits/stdc++.h> 

// mutex for multithreading
pthread_cond_t condition = PTHREAD_COND_INITIALIZER; 
pthread_cond_t condition_2 = PTHREAD_COND_INITIALIZER; 
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; 

//global variable used in the three threads
MbsData *mbs_data;
int robotran_finish = 0;

void give_torque_access();

void give_pos_vit_access();

void give_torque_access() //mutex to "give the hand" to the thread wich publish the torque of the steerwheel
{
	//access for torque
	 pthread_mutex_lock (&mutex); 
     pthread_cond_signal(&condition); 
     pthread_mutex_unlock (&mutex);
}

void give_pos_vit_access() //mutex to "give the hand" to the thread wich listen to the postion and velocity of the steerwheel
{
   //access for pos and vit
     pthread_mutex_lock (&mutex); 
     pthread_cond_signal(&condition_2); 
     pthread_mutex_unlock (&mutex);
}

//callback function for the topic chatter_pos_vit wich update the position and velocity of the steerwheel
// Change "R1_steerwheel_id" with the joint ID of the steerwheel of your vehicle
// Set the maximum angle of your steerwheel
void chatterCallback(const ros_rob::Pos_vit_msg& msg) 
{
	   double max_angle = 8.0;
       mbs_data->q[R1_steerwheel_id] = msg.pos_value;
       if(mbs_data->q[R1_steerwheel_id] > max_angle)
       {
        mbs_data->q[R1_steerwheel_id] = max_angle;
       }
       else if(mbs_data->q[R1_steerwheel_id] < -max_angle)
       {
        mbs_data->q[R1_steerwheel_id] = -max_angle;
       }
       mbs_data->qd[R1_steerwheel_id] = msg.vit_value; 
}

//thread listener (for the position and velocity of the steeerwheel)
void *ros_posvit_thread_3(void *arg_data_3) 
{
	sleep(1); 

	ros::NodeHandle pos_vit_lis;
   
 
	 ros::Subscriber sub_pos = pos_vit_lis.subscribe("chatter_pos_vit", 1, chatterCallback);    

    while (robotran_finish != 1)
     {
     	pthread_mutex_lock(&mutex); 
        pthread_cond_wait(&condition_2,&mutex);
   
        ros::spinOnce(); //call chatterCallback once

        pthread_mutex_unlock (&mutex);
     }

    printf("ROS Pos & Vit finish ! \n");
    pthread_exit(NULL);
}


//thread publisher (for the torque of the steerwheel)
// Change "R1_steerwheel_id" with the joint ID of the steerwheel of your vehicle
void *ros_torque_thread_1(void *arg_data_1) 
{
  
     ros::NodeHandle torque_pub;

     ros::Publisher chatter_pub = torque_pub.advertise<ros_rob::Torque_msg>("chatter_torque", 1); 

     ros_rob::Torque_msg torque;

     double torque_value;
  
     double torque_init = 1030; //[Ncm], continuous torque at standstill of the servo-motor 

     
      while (robotran_finish != 1)
        {
             pthread_mutex_lock(&mutex); 

		         pthread_cond_wait(&condition, &mutex);

             torque_value = -mbs_data->Qc[R1_steerwheel_id];  // Change "R1_steerwheel_id" with the joint ID of the steerwheel of your vehicle          

             pthread_mutex_unlock (&mutex);

            // ROS_INFO("Couple = %f", torque.value);
             
             //command of the torque in percent of continuous torque at standstill of the servo-motor 
             //absolute maximum torque = 30.9 Nm (300 % of 1030 Ncm)
             torque.value = std::min(300.0,(std::max(-300.0,(torque_value*10000.0)/torque_init))); 
            
             chatter_pub.publish(torque);
        }


     printf("ROS Torque finish ! \n");

     pthread_exit(NULL);
}

void *robotran_thread_2(void *arg_data_2) //thread for the Robotran simulation 
{   
	ros::NodeHandle node_ros("~");
    
    sleep(1); //to let the two other threads reach the wait condition 
  
    MbsPart *mbs_part;
    MbsDirdyn *mbs_dirdyn;
    MbsEquil *mbs_equil;

    ROS_INFO("Start Robotran");

    printf("Starting Car MBS project!\n");


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                     LOADING                               *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    
    printf("Loading the Car data file !\n");
    //set the correct path to your .mbs file
    mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/Car.mbs", BUILD_PATH);
    printf("*.mbs file loaded!\n");

    ThreadStruct *thread_struct; //user structure to store pointer to the give_access function 

    thread_struct = (ThreadStruct*)malloc(sizeof(ThreadStruct));

    thread_struct->pointeur_give_torque_access = give_torque_access;

    thread_struct->pointeur_give_pos_vit_access = give_pos_vit_access;

    mbs_data->user_model->thread.thread_struct = thread_struct; 

    
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                     CONSTRAINT                            *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    
    //In case of user constraints in your model, insert the initialisation here
	/* Example : 
	int N_usr_c = 6;
	mbs_set_nb_userc(mbs_data, N_usr_c);
    */

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*              COORDINATE PARTITIONING                      *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
   
     // Insert the coordinate partitioning module here 

    /* * * * * * * * * * * * * * *  * * * * * * * *
    /*                STATIC EQUILIBRIUM          *
    /* * * * * * * * * * * * * * ** * * * * * * * */
    

    // Insert the static equilibrium module here

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                STRAIGHT LINE  EQUILIBRIUM                 *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

 
    //ros param to set the initiale speed when using the launch_haptic_simulator.py function, set to 30 km/h by default
    double V;
    node_ros.param("speed", V, 30.0); 
    V = V/3.6;
   
    //Insert the quasistatic equilibrium here

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                   DIRECT DYNANMICS                        *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	
    mbs_set_qdriven(mbs_data, R1_steerwheel_id); // Change "R1_steerwheel_id" with the joint ID of the steerwheel of your vehicle       

    //create your mbs_dirdyn (direct dynamique) et set the options here
    //Don't forget to enable the real_time option 
    /*Example 
    mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // dirdyn options (see documentations for additional options)
    mbs_dirdyn->options->dt0 = 0.82*1e-3; 
    mbs_dirdyn->options->tf  = 10.0;
    mbs_dirdyn->options->save2file = 1;
    mbs_dirdyn->options->realtime = 1;
    */
    
    robotran_finish = 0;
     
    // in order to wait that the servo-drive is set for the haptic simulator
    int haptic_ready = 0;
    ros::param::get("haptic_ready", haptic_ready);
    while(haptic_ready == 0)
    {
       ros::param::get("haptic_ready", haptic_ready);
    }

    ros::param::set("robotran_simu_run", 1); //to advertise the servo-drive that the Robotran simulation is running

    //initialise run the direct dynamique here
    /*Example
     mbs_dirdyn_init(mbs_dirdyn, mbs_data);
      mbs_dirdyn_loop(mbs_dirdyn, mbs_data);
    */

    ros::param::set("robotran_simu_run", 0); //to advertise the servo-drive that the Robotran simulation is finish 

    //finish and delete your mbs_dirdyn here
    /* Example
    mbs_dirdyn_finish(mbs_dirdyn, mbs_data);
    mbs_delete_dirdyn(mbs_dirdyn, mbs_data);
    */
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                   CLOSING OPERATIONS                      *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    

    free(thread_struct);

    pthread_mutex_lock (&mutex); 
    robotran_finish = 1; //advertise the two other threads that the simulation is finish
    pthread_cond_signal(&condition); 
    pthread_cond_signal(&condition_2); 
    pthread_mutex_unlock (&mutex);

    sleep(1); 
    give_torque_access(); //in order to be sure that the two other threads are succesfully closed 
    give_pos_vit_access(); 

    mbs_delete_data(mbs_data);
    printf("Robotran finish ! \n");


    pthread_exit(NULL);
}




int main(int argc, char *argv[])
{
     pthread_t thread1; //ROS_torque thread
     pthread_t thread2; //Robotran thread
     pthread_t thread3; //ROS_posvit thread

     ros::init(argc, argv, "rosbotran");

     ros::NodeHandle node;
       
     ros::param::set("robotran_finish", 0);
   
    if(pthread_create(&thread2, NULL, robotran_thread_2, NULL) == 0  && pthread_create(&thread1, NULL, ros_torque_thread_1, NULL) == 0 && pthread_create(&thread3, NULL, ros_posvit_thread_3, NULL) == 0)
    {

          pthread_join(thread1, NULL);
          pthread_join(thread2, NULL);
          pthread_join(thread3, NULL);

    }
    else
    {
    	printf("Error : thread not created");

    	return -1;
    }
  
    pthread_mutex_destroy(&mutex);


    //giving choice to restart a new simulation when using the launch_haptic_simulator.py function 
    char restart_choice;
     ros::param::set("choice_done", 0);
    do
    {
    printf("--------------------------------------------------------------------\n");
    std::cout << "The simulation is finish. Would you like to start another one ? [Y/N] : ";
    std::cin >> restart_choice;
    } 
     while( !std::cin.fail() && restart_choice !='Y' &&  restart_choice !='N'  && restart_choice !='y' && restart_choice !='n');

    if(restart_choice =='Y' ||  restart_choice =='y')
    {
       printf("Restart a new one ... \n");
       ros::param::set("restart_robotran", 1);
       ros::param::set("choice_done", 1);
       ros::param::set("robotran_simu_run", 1);
    }
    else 
    {
       printf("End of the simulation ... \n");
       ros::param::set("restart_robotran", 0);
       ros::param::set("choice_done", 1);
       ros::param::set("robotran_simu_run", 1);
    }
    
     ros::param::set("robotran_finish", 1);

     sleep(1);
     ros::param::set("choice_done", 0);
  
    printf("Code finish \n");

    return 0;
}

