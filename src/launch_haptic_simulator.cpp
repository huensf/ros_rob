#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>

using namespace std;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  //ros::init(argc, argv, "launcher");

  printf("     ||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n");
  printf("     ||                                                    ||\n");
  printf("     ||  WELCOME TO THE VEHICLE HAPTIC FEEDBACK SIMULATOR  ||\n");
  printf("     ||                                                    ||\n");
  printf("     ||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n");

  printf("--------------------------------------------------------------------\n");
  printf("Choose your vehicle: \n");
  printf(" 1. Car \n");
  printf(" 2. Kart\n");
      int vehicle_choice;
     do
     {
    cout << "Enter the number of the vehicle you want : ";
    cin >> vehicle_choice;
    } 
     while( !cin.fail() && vehicle_choice !=1 &&  vehicle_choice !=2);

    if(vehicle_choice == 1)
    {
       printf("The \"Car\" has been selected ! \n");
       system("rosrun ros_rob exe_Car");
    }
    else if (vehicle_choice == 2)
    {
       printf("The \"Kart\" has been selected ! \n");
       system("rosrun ros_rob exe_KART_V2");
    }


 printf("--------------------------------------------------------------------\n");
 printf(" Launching the simulation ... \n");
 // system("ssh raspberry_ethernet rosrun ros_rasp lexium32A_CANopen");
 system("ssh raspberry_ethernet rosrun ros_rasp talker");

 printf("--------------------------------------------------------------------\n");
    

  return 0;
}