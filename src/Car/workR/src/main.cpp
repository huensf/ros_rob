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

// ROS and topic's msg
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_rob/Torque_msg.h"
#include "ros_rob/Pos_vit_msg.h"
#include "ros_rob/Pedals_msg.h"
#include "std_msgs/UInt8MultiArray.h"

#include <sstream>
#include <ctime>

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

void give_pos_vit_ped_access();

void give_torque_access() //mutex to "give the hand" to the thread wich publish the torque of the steerwheel
{
	//access for torque
	 pthread_mutex_lock (&mutex); 
     pthread_cond_signal(&condition); 
     pthread_mutex_unlock (&mutex);
}

void give_pos_vit_ped_access() //mutex to "give the hand" to the thread wich listen to the postion and velocity of the steerwheel and the pedals position
{
   //access for pos and vit and ped
     pthread_mutex_lock (&mutex); 
     pthread_cond_signal(&condition_2); 
     pthread_mutex_unlock (&mutex);
}

void chatterCallback_pos_vit(const ros_rob::Pos_vit_msg& msg) //callback function for the topic chatter_pos_vit, update the postion and velocity of the steerwheel
{
    mbs_data->q[R1_steerwheel_id] = msg.pos_value;
    if(mbs_data->q[R1_steerwheel_id] > 9)
        mbs_data->q[R1_steerwheel_id] = 9;
    else if(mbs_data->q[R1_steerwheel_id] < -9)
        mbs_data->q[R1_steerwheel_id] = -9;
    mbs_data->qd[R1_steerwheel_id] = msg.vit_value; 
}

void chatterCallback_pedals(const ros_rob::Pedals_msg& msg) //callback function for the topic chatter_pedals, update the speed of the vehicle
{
  mbs_data->user_model->pedals.ped1 = msg.pedal1_value;
  mbs_data->user_model->pedals.ped2 = msg.pedal2_value;
  printf("ped1(acc): %f\tped2(brake): %f\n", msg.pedal1_value, msg.pedal2_value); 
}

void *ros_posvit_thread_3(void *arg_data_3) //thread listener (for the position and velocity of the steeerwheel)
{
	sleep(1); 

	ros::NodeHandle pos_vit_lis;
	ros::NodeHandle pedals_lis;
	
	ros::Subscriber sub_pos = pos_vit_lis.subscribe("chatter_pos_vit", 1, chatterCallback_pos_vit); 
	ros::Subscriber sub_ped = pedals_lis.subscribe("chatter_pedals", 1, chatterCallback_pedals);   

    while (robotran_finish != 1)
    {
     	pthread_mutex_lock(&mutex); 
        pthread_cond_wait(&condition_2,&mutex);
   
        ros::spinOnce(); //call chatterCallback once

        pthread_mutex_unlock (&mutex);
    }

    printf("ROS Pos & Vit + Pedals finish ! \n");
    pthread_exit(NULL);
}



void *ros_torque_thread_1(void *arg_data_1) //thread publisher (for the torque of the steerwheel)
{
    ros::NodeHandle torque_pub;

    ros::Publisher chatter_pub = torque_pub.advertise<ros_rob::Torque_msg>("chatter_torque", 1); 

    ros_rob::Torque_msg torque;

    double torque_value;
    double torque_init = 10.3;  //100*[Ncm], continuous torque at standstill of the servo-motor
    double speed;
      
    int count = 0;

    while (robotran_finish != 1)
    {
        pthread_mutex_lock(&mutex); 
        pthread_cond_wait(&condition, &mutex);
		     
		// damping for the steerwheel torque
		speed = 3.6 * sqrt(pow(mbs_data->qd[T1_chassis_id],2)+pow(mbs_data->qd[T2_chassis_id],2));
		if (speed <= 30.0)
            mbs_data->user_model->steerwheel.D = 0.6;
        else if (speed > 30 && speed <= 50)
            mbs_data->user_model->steerwheel.D = 0.8;
        else if (speed > 50 && speed <= 70)
            mbs_data->user_model->steerwheel.D = 1.5;
        else 
            mbs_data->user_model->steerwheel.D = 2;

        //torque_value = mbs_data->Qc[R1_steerwheel_id];
        torque_value = mbs_data->Qc[R1_steerwheel_id] + mbs_data->user_model->steerwheel.D * mbs_data->qd[R1_steerwheel_id];
        pthread_mutex_unlock (&mutex);

        // ROS_INFO("Couple = %f", torque.value);
             
        //command of the torque in percent of continuous torque at standstill of the servo-motor 
        //absolute maximum torque : 30.9 Nm (300 % of 1030 Ncm)
        //torque.value = std::min(300.0,(std::max(-300.0,(torque_value*10000.0)/torque_init))); 
        torque.value = 0.01*std::min(300.0,(std::max(-300.0,(torque_value*10000.0)/torque_init)));

        //send_torque_pdo(m,torque_can_msg,-270,chatter_pub);
        chatter_pub.publish(torque);
    }
     
    // to stop the steerwheel after the simu 
    std::clock_t tac = std::clock();
    while ((std::clock() - tac) / (double)CLOCKS_PER_SEC < 1)
    {
        torque.value = 0.0;
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

    int simulation_choice = 1;
    ros::param::get("simulation_choice", simulation_choice);
    
    printf("Loading the Car data file !\n");

    if(simulation_choice == 1)
    {
      printf("test\n");
      mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/Car.mbs", BUILD_PATH);
      printf("test\n");
    }
    else if(simulation_choice == 2)
    {
      mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/Car_a.mbs", BUILD_PATH);
    }
    else if(simulation_choice == 3)
    {
      mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/Car_b.mbs", BUILD_PATH);

       //variable tail
      time_t t_2;
      srand((unsigned) time(&t_2));

      mbs_data->dpt[1][Fusee_av_d_1_to_roue_id] = 0.003-((double)(rand() % 14)/1000.0); 
      mbs_data->dpt[1][Fusee_av_g_1_to_roue_id] =  mbs_data->dpt[1][Fusee_av_d_1_to_roue_id];

      printf("Point ancrage pour chasse : %f \n", mbs_data->dpt[1][Fusee_av_d_1_to_roue_id]);
    }
    else if(simulation_choice == 4)
    {
      mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/Car_c.mbs", BUILD_PATH);

      //random number for slipping in traffic circle
      time_t t;
      srand((unsigned) time(&t));
      mbs_data->user_model->expe.t_rand = 12+ (rand() % 11); 

       //variable tail
      time_t t_2;
      srand((unsigned) time(&t_2));

      mbs_data->dpt[1][Fusee_av_d_1_to_roue_id] = 0.003-((double)(rand() % 14)/1000.0); 
      mbs_data->dpt[1][Fusee_av_g_1_to_roue_id] =  mbs_data->dpt[1][Fusee_av_d_1_to_roue_id];

      printf("Point ancrage pour chasse : %f \n", mbs_data->dpt[1][Fusee_av_d_1_to_roue_id]);
    }
    else if(simulation_choice == 5)
    {
      mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/Car_d.mbs", BUILD_PATH);

      //random number for slipping in traffic circle
      time_t t;
      srand((unsigned) time(&t));
      mbs_data->q[T1_chassis_id] = (rand() % 51); 

       //variable tail
      time_t t_2;
      srand((unsigned) time(&t_2));

      mbs_data->dpt[1][Fusee_av_d_1_to_roue_id] = 0.003-((double)(rand() % 14)/1000.0); 
      mbs_data->dpt[1][Fusee_av_g_1_to_roue_id] =  mbs_data->dpt[1][Fusee_av_d_1_to_roue_id];

      printf("Point ancrage pour chasse : %f \n", mbs_data->dpt[1][Fusee_av_d_1_to_roue_id]);
    }
    else if(simulation_choice == 6)
    {
      mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/Car_simple-turn.mbs", BUILD_PATH);
    }
    else if(simulation_choice == 7)
    {
      mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/Car_2lines.mbs", BUILD_PATH);
    }

    printf("*.mbs file loaded!\n");

    ThreadStruct *thread_struct; //user structure to store pointer to the give_access function 

    thread_struct = (ThreadStruct*)malloc(sizeof(ThreadStruct));

    thread_struct->pointeur_give_torque_access = give_torque_access;

    thread_struct->pointeur_give_pos_vit_ped_access = give_pos_vit_ped_access;

    mbs_data->user_model->thread.thread_struct = thread_struct; 

    // regulator PID for the torque of the rear wheels in order to have constant speed
    mbs_data->user_model->PID.Kp = 150;//250.0;

    mbs_data->user_model->PID.Ki = 0.00;

    mbs_data->user_model->PID.Kd = 100;

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                     CONSTRAINT                            *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	int N_usr_c = 6;
	mbs_set_nb_userc(mbs_data, N_usr_c);

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*              COORDINATE PARTITIONING                      *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
   
    mbs_part = mbs_new_part(mbs_data);
    mbs_part->options->rowperm=1;
    mbs_part->options->verbose = 0;
    mbs_run_part(mbs_part, mbs_data);
    mbs_delete_part(mbs_part);

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                STATIC EQUILIBRIUM at given height         *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    

  //  mbs_data->dpt[2][47] = -0.559461; //wheel orientation (doesn't work)
   // mbs_data->dpt[2][48] = 0.54;
   
    mbs_data->process = 1;
    mbs_equil = mbs_new_equil(mbs_data);
    // equil options (see documentations for additional options)
    mbs_equil->options->senstol = 1e-2; //1e-2
    mbs_equil->options->devjac = 1e-2; //1e-2
	//mbs_equil->options->equitol = 7e-4;
    mbs_equil->options->verbose = 0;
    mbs_equil->options->save2file = 0;
    mbs_equil->options->resfilename = (char*)"static_1";

    mbs_data->q[R2_chassis_id] = 0.0;
    mbs_data->q[T3_chassis_id] = 0.2;


    // --- Variable exchange, quch->xch
    mbs_equil->options->nquch = 2; // nquch = nxch number of exchanged variables
    mbs_equil_exchange(mbs_equil->options); // allocates the memory
    mbs_equil->options->quch[1] = T3_chassis_id; // which free coordinate has to be replaced
    mbs_equil->options->xch_ptr[1] = &(mbs_data->user_model->FrontSuspension.L0); // which variable is the new one
    mbs_equil->options->quch[2] = R2_chassis_id;
    mbs_equil->options->xch_ptr[2] = &(mbs_data->user_model->RearSuspension.L0);

    // equilibrium procedure
    mbs_run_equil(mbs_equil, mbs_data);
    mbs_print_equil(mbs_equil);
    mbs_delete_equil(mbs_equil, mbs_data); //mettre en commentaire si on veut faire wheel orientation


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                STRAIGHT LINE  EQUILIBRIUM                 *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    double V;
    node_ros.param("speed", V, 30.0); //ros param to set the initiale speed when using the launch_haptic_simulator.py function, set to 30 km/h by default

    V = V/3.6;
   
    // intialisation of the PID
    mbs_data->user_model->PID.velocity = V;
    mbs_data->user_model->PID.sum_error = 0;
    mbs_data->user_model->PID.previous_error = 0;


   
    printf("-----------------Straight line equil-----------------\n");


    mbs_data->process = 2;
    mbs_equil = mbs_new_equil(mbs_data);
    // equil options (see documentations for additional options)
    mbs_equil->options->senstol = 1e-06; 
    mbs_equil->options->devjac = 1e-05;
    mbs_equil->options->equitol = 1.5; //DIMINUER LE RESIDU (OLD : 4.5)
//	mbs_equil->options->itermax = 100;
    mbs_equil->options->verbose = 0;
    mbs_equil->options->mode = 2;
    mbs_equil->options->save2file = 0;
    mbs_equil->options->resfilename = (char*)"sl";

    // set pedals_mode
    int pedals_mode = 0;
    ros::param::get("pedals_mode", pedals_mode);
    mbs_data->user_model->pedals.mode = pedals_mode;


    // set a desired speed
    mbs_data->qd[T1_chassis_id] = V; //[m/s]
                                         // calculation of the true height of the center of the wheel = R_wheel (0.3) - penetration
                                         // see in ExtForces : printf("%f\n", PxF[3]); height of the sensor
    mbs_data->qd[R2_wheel_ft_rt_id] = V / 0.279422;
    mbs_data->qd[R2_wheel_ft_lt_id] = V / 0.279422;
    mbs_data->qd[R2_wheel_rr_lt_id] = V / 0.280650;
    mbs_data->qd[R2_wheel_rr_rt_id] = V / 0.280650;
    // --- Variable exchange, quch->xch
    mbs_equil->options->nquch = 5; // nquch = nxch number of exchanged variables
    mbs_equil_exchange(mbs_equil->options); // allocates the memory
    mbs_equil->options->quch[1] = T1_chassis_id; // which free coordinate has to be replaced
    mbs_equil->options->xch_ptr[1] = &(mbs_data->user_model->EquilQuantities.Qpropulsion); // which variable is the new one
    mbs_equil->options->quch[2] = R2_wheel_ft_rt_id;
    mbs_equil->options->xch_ptr[2] = &(mbs_data->qd[R2_wheel_ft_rt_id]);
    mbs_equil->options->quch[3] = R2_wheel_ft_lt_id;
    mbs_equil->options->xch_ptr[3] = &(mbs_data->qd[R2_wheel_ft_lt_id]);
    mbs_equil->options->quch[4] = R2_wheel_rr_lt_id;
    mbs_equil->options->xch_ptr[4] = &(mbs_data->qd[R2_wheel_rr_lt_id]);
    mbs_equil->options->quch[5] = R2_wheel_rr_rt_id;
    mbs_equil->options->xch_ptr[5] = &(mbs_data->qd[R2_wheel_rr_rt_id]);
    // equilibrium procedure
    mbs_run_equil(mbs_equil, mbs_data);
    mbs_print_equil(mbs_equil);

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                   DIRECT DYNANMICS                        *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	
	int vue_choice = 0;
  ros::param::get("vue_choice", vue_choice);
  mbs_data->user_model->command_key.torque = vue_choice;   

  mbs_set_qdriven(mbs_data, R1_steerwheel_id); // qu ou qdriven

  mbs_data->process = 2; 
  mbs_dirdyn = mbs_new_dirdyn(mbs_data);

  // dirdyn options (see documentations for additional options)
  mbs_dirdyn->options->dt0 = 0.82 * 1e-3; 

  mbs_dirdyn->options->tf = 10.0;
  if(simulation_choice == 1)
    mbs_dirdyn->options->tf = 90.0;
  else if(simulation_choice == 2)
    mbs_dirdyn->options->tf = 160.0;
  else if(simulation_choice == 3)
    mbs_dirdyn->options->tf = 20.0;
  else if(simulation_choice == 4)
    mbs_dirdyn->options->tf = 30.0;
  else if(simulation_choice == 5)
    mbs_dirdyn->options->tf = 15.0;
  else if(simulation_choice == 6)
  	mbs_dirdyn->options->tf = 15.0;
  else if(simulation_choice == 7)
  	mbs_dirdyn->options->tf = 120.0;

  mbs_dirdyn->options->save2file = 1;
  mbs_dirdyn->options->realtime = 1;

  mbs_dirdyn->options->respath = PROJECT_SOURCE_DIR"/../resultsR";
 
  robotran_finish = 0;

  // in order to wait that the servo-drive is set for the haptic simulator
  /*int haptic_ready = 0;
  ros::param::set("haptic_ready", haptic_ready);
  while(haptic_ready == 0)
  {
     ros::param::get("haptic_ready", haptic_ready);
  }*/

  ros::param::set("robotran_simu_run", 1); //to advertise the servo-drive that the Robotran simulation is running

  mbs_dirdyn_init(mbs_dirdyn, mbs_data);
  mbs_dirdyn_loop(mbs_dirdyn, mbs_data);

  ros::param::set("robotran_simu_run", 0); //to advertise the servo-drive that the Robotran simulation is finish 

  mbs_dirdyn_finish(mbs_dirdyn, mbs_data);
    
  mbs_delete_dirdyn(mbs_dirdyn, mbs_data);
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
  give_pos_vit_ped_access(); 

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
    /*char restart_choice;
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
      ros::param::set("robotran_finish", 0);
      ros::param::set("choice_done", 1);
      ros::param::set("robotran_simu_run", 1);
    }
    else 
    {*/
      printf("End of the simulation ... \n");
      ros::param::set("restart_robotran", 0);
      ros::param::set("choice_done", 1);
      ros::param::set("robotran_simu_run", 1);
    //}
    
    ros::param::set("robotran_finish", 1);
    sleep(1);
    ros::param::set("choice_done", 0);
  
    printf("Code finish \n");

    return 0;
}

