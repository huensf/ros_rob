#!/usr/bin/env python

import rospy
import roslaunch
import rosparam
import sys
import os

def start_task():

  rospy.init_node('launch_node')

  rosparam.set_param('test', '2.0')


  print("")
  print("     ||||||||||||||||||||||||||||||||||||||||||||||||||||||||")
  print("     ||                                                    ||")
  print("     ||  WELCOME TO THE VEHICLE HAPTIC FEEDBACK SIMULATOR  ||")
  print("     ||                                                    ||")
  print("     ||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n")

  print("--------------------------------------------------------------------\n")
  print(" Launching the process on the Raspberry  ... \n")

  restart_robotran = 1

  while restart_robotran == 1:


    print("--------------------------------------------------------------------\n")
    print("Choose your vehicle: \n")
    print(" 1. Car \n")
    print(" 2. Kart \n")  
    print 'Enter the number of the vehicle you want :',
    
  #SELECT THE VEHICLE
    valid_choice = False
    package = 'ros_rob' 
    required = 'False'
    name = 'Simulation_Robotran'

    while valid_choice == False:

      vehicle_choice = raw_input().lower()

      if vehicle_choice == '1':
    
         print("")
         print("The \"Car\" has been selected ! \n")
       
         node_type = 'exe_Car'
       
         valid_choice = True

         print("--------------------------------------------------------------------\n")
         print("Do you want to use the pedals ? \n")
         print(" 1. YES - Info: you will be able to choose the initial speed\n");
         print(" 2. NO (constant speed)\n");
         print 'Enter the number corresponding to your choice :',

         valid_choice_1 = False

         while valid_choice_1 == False:

         	pedals_mode = raw_input().lower()

         	if pedals_mode == '1' or pedals_mode == '2':

         		rosparam.set_param('pedals_mode', pedals_mode)

         		valid_choice_1 = True

         	else :

         		print 'Please enter a valid choice :',

         print("--------------------------------------------------------------------\n")
         print("Choose your type of simulation: \n")
         print(" 1. Circuit 1 (Advised speed if constant : 50 km/h)\n")
         print(" 2. Circuit 2 (Advised speed if constant : 50 km/h)\n")
         print(" 3. Full speed turn (Advised speed if constant : 150 km/h)\n")
         print(" 4. Traffic circle  (Advised speed if constant : 40 km/h)\n")
         print(" 5. Obstacle avoid (Advised speed if constant : 75 km/h)\n")
         print(" 6. Simple turn (Advised speed if constant : 50 km/h)\n")
         print(" 7. Straight lines\n")
         print 'Enter the number of the simulation you want :',
     
         valid_choice_2 = False

         while valid_choice_2 == False:

          simulation_choice = raw_input().lower()

          if simulation_choice == '1' or simulation_choice == '2' or simulation_choice == '3' or simulation_choice == '4' or simulation_choice == '5' or simulation_choice == '6' or simulation_choice == '7':

            rosparam.set_param('simulation_choice', simulation_choice)

            valid_choice_2 = True

          else : 

            print 'Please enter a valid choice :',

         print("--------------------------------------------------------------------\n")
         print("Choose your simulation view: \n")
         print(" 0. Outside the car\n")
         print(" 5. Inside the car\n")
         print 'Enter the number of the simulation view you want :',
     
         valid_choice_3 = False

         while valid_choice_3 == False:

          vue_choice = raw_input().lower()

          if vue_choice == '0' or vue_choice == '5':

            rosparam.set_param('vue_choice', vue_choice)

            valid_choice_3 = True

          else : 

            print 'Please enter a valid choice :',

    
      elif vehicle_choice == '2':

         print("")
         print("The \"Kart\" has been selected ! \n")

         node_type = 'exe_KART'
       
         valid_choice = True

         print("--------------------------------------------------------------------\n")
         print("Do you want to use the pedals ? \n")
         print(" 1. YES - Info: you will be able to choose the initial speed\n");
         print(" 2. NO (constant speed)\n");
         print 'Enter the number corresponding to your choice :',

         valid_choice_1 = False

         while valid_choice_1 == False:

         	pedals_mode = raw_input().lower()

         	if pedals_mode == '1' or pedals_mode == '2':

         		rosparam.set_param('pedals_mode', pedals_mode)

         		valid_choice_1 = True

         	else :

         		print 'Please enter a valid choice :',

         print("--------------------------------------------------------------------\n")
         print("Choose your type of simulation: \n")
         print(" 1. Circuit 1 (Advised speed = 45 km/h)\n")
         print(" 2. Circuit 2 (Advised speed = 45 km/h)\n")
         print 'Enter the number of the simulation you want :',
     
         valid_choice_2 = False

         while valid_choice_2 == False:

          simulation_choice = raw_input().lower()

          if simulation_choice == '1' or simulation_choice == '2':

            rosparam.set_param('simulation_choice', simulation_choice)

            valid_choice_2 = True

          else : 

            print 'Please enter a valid choice :',

      else:

          print 'Please enter a valid choice :',

  #SELECT THE SPEED
    valid_choice = False
    print("--------------------------------------------------------------------\n")
    print 'Enter the initial speed of your vehicle in km/h :',

    while valid_choice == False:

      speed_choice = raw_input().lower()

      if float(speed_choice) > 0 and float(speed_choice) < 151:
    
         print("")
         print("Your initial speed is set to "+speed_choice+" km/h \n")
       
         args = '_speed:='+speed_choice
       
         valid_choice = True
    
      else:

          print 'Please enter a value between 1 and 150 :',

        
    
    node = roslaunch.core.Node(package, node_type, name, '/', None, args, False, 0.0, None, None, 'screen' , None, None, False)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)
    #print process.is_alive()

    

    rosparam.set_param("robotran_simu_run", '0')
    rospy.sleep(0.2)
    rosparam.set_param("robotran_simu_run", '1')
    robotran_finish = rospy.get_param("robotran_finish",0)
    while robotran_finish == 0:
        rospy.sleep(1)
        robotran_finish = rospy.get_param("robotran_finish")
    rosparam.set_param("robotran_simu_run", '1')

    restart_robotran = rospy.get_param("restart_robotran")
    rosparam.set_param('robotran_finish', '0')
    process.stop()
    rospy.sleep(2)
    

  

  print("--------------------------------------------------------------------\n")

  

    # package = 'ros_rob' 
    # executable = 'exe_Car'
    # node = roslaunch.core.Node(package, executable)

     #launch = roslaunch.scriptapi.ROSLaunch()
     #launch.start()

     #process = launch.launch(node)
     #print process.is_alive()


    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/labo/catkin_ws/src/ros_rob/launch/test.launch"])
    # launch.start()

     #rospy.sleep(10)
     #launch.shutdown()
def main():

	start_task()
	#rospy.spin()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
