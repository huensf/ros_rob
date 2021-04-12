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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_rob/Torque_msg.h"
#include "ros_rob/Pos_vit_msg.h"
#include <sstream>

#include "pthread.h"

int robotran_finish = 0;

pthread_cond_t condition = PTHREAD_COND_INITIALIZER; 
pthread_cond_t condition_2 = PTHREAD_COND_INITIALIZER; 
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; 

MbsData *mbs_data;

void give_torque_access();

void give_pos_vit_access();

void give_torque_access()
{
	//access for torque
	 pthread_mutex_lock (&mutex); 
     pthread_cond_signal(&condition); 
     pthread_mutex_unlock (&mutex);
}

void give_pos_vit_access()
{
   //access for pos and vit
     pthread_mutex_lock (&mutex); 
     pthread_cond_signal(&condition_2); 
     pthread_mutex_unlock (&mutex);
}

void chatterCallback(const ros_rob::Pos_vit_msg& msg)
{

  pthread_mutex_lock(&mutex); 
  pthread_cond_wait(&condition_2,&mutex);

 // ROS_INFO("Position : [%f]", msg.pos_value);
  // ROS_INFO("Vitesse : [%f]", msg.vit_value);

  //SET THE JOINT ID OF YOUR MBS MODEL
  //mbs_data->q[50] = msg.pos_value; decommenter quand CAN Communication OK 
  //mbs_data->qd[50] = msg.vit_value;

  pthread_mutex_unlock (&mutex);

  if(robotran_finish==1)
  {
  	printf("ROS Pos & Vit finish ! \n");
  	pthread_exit(NULL);
  }

}

void *ros_posvit_thread_3(void *arg_data_3)
{
	sleep(1); 

	ros::NodeHandle pos_vit_lis;
   
    ros::Subscriber sub_pos = pos_vit_lis.subscribe("chatter_pos_vit", 1000, chatterCallback); 
  
    ros::spin();

    pthread_exit(NULL);
}



void *ros_torque_thread_1(void *arg_data_1)
{
  
     ros::NodeHandle torque_pub;

     ros::Publisher chatter_pub = torque_pub.advertise<ros_rob::Torque_msg>("chatter_torque", 1000); 
    
      while (robotran_finish != 1)
        {

        	 ros_rob::Torque_msg torque;

             pthread_mutex_lock(&mutex); 

		     pthread_cond_wait(&condition, &mutex);

             //SET THE JOINT ID OF YOUR MBS MODEL
             torque.value = mbs_data->Qc[50];            

             pthread_mutex_unlock (&mutex);

            // ROS_INFO("Couple = %f", torque.value);

             chatter_pub.publish(torque);

        }

     printf("ROS Torque finish ! \n");

     pthread_exit(NULL);
}

void *robotran_thread_2(void *arg_data_2)
{   
	sleep(1); //pour arriver a la condition wait

    //[INSERT YOUR ROBOTRAN MAIN HERE]
	// WITHOUT MbsData *mbs_data; (already declared as a global variable)
	// WITHOUT return 0; AT THE END
     
    //[ADD THIS LINES AT THE END OF THE LOADING MODULE]
    
    ThreadStruct* thread_struct; 

    thread_struct = (ThreadStruct*)malloc(sizeof(ThreadStruct));

    thread_struct->pointeur_give_torque_access = give_torque_access;

    thread_struct->pointeur_give_pos_vit_access = give_pos_vit_access;

    mbs_data->user_model->thread.thread_struct = thread_struct; 
    
    //DON'T FORGET TO SET ON THE REAL-TIME OPTION

    //[ADD THIS LINES AT THE END OF THE MAIN]

    free(thread_struct);

    pthread_mutex_lock (&mutex); 
    robotran_finish = 1;
    pthread_cond_signal(&condition); 
    pthread_cond_signal(&condition_2); 
    pthread_mutex_unlock (&mutex);

    printf("Robotran finish ! \n");

    sleep(1); 
    give_ros_access(); //pour etre sur que les threads ROS se finissent correctement

    pthread_exit(NULL);
}




int main(int argc, char *argv[])
{
     pthread_t thread1; //ROS_torque thread
     pthread_t thread2; //Robotran thread
     pthread_t thread3; //ROS_posvit thread

     ros::init(argc, argv, "rosbotran");

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
  
    printf("Code finish \n");

    return 0;
}

