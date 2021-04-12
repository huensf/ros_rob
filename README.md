# Ros_rob
This is a ROS package that allows you to run a Robotran project and publish/receive data on ROS topics. It is complementary to [*ros_rasp*](https://git.immc.ucl.ac.be/huensf/ros_rasp) and [*ros_rasp2*](https://git.immc.ucl.ac.be/huensf/ros_rasp2).

Here specially is an example where you can drive a car with a steerwheel: the position and velocity of this latter are send on a ROS topic and the the torque in the steerwheel is received via another one. A third topic allows to receive data about the two pedals with which you can modify the speed of the vehicle.

[[_TOC_]]

## Installation and configuration of ROS 
For installing ROS follow [this link](http://wiki.ros.org/noetic/Installation/Ubuntu). To configure your ROS environnement follow the [Installation and Configuration](http://wiki.ros.org/fr/ROS/Tutorials/InstallingandConfiguringROSEnvironment) with one constraint, you must create your catkin workspace (*catkin_ws*) in your */home/user_name/* folder (in order to allow Robotran to find the correct path to an executable *open_gl_exe* mandatory for the real-time simulation). 

After that, in order to have a better comprehension and management of ROS, you can follow all the beginner level from [official ROS Tutorials](http://wiki.ros.org/fr/ROS/Tutorials) and [Writing a simple publisher/sucriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).  

Note that this ROS package has been written and tested on Linux distribution. 

The ROS versions used here are *Melodic* for the PC (master) and *Noetic* for the Raspberry's (slaves) and the ROS compiler used is *catkin*. The nodes are written in C++.
 
## Installation and configuration of Robotran
For using Robotran on Linux you need to install [MBsysPad](http://robotran-doc.git-page.immc.ucl.ac.be/Installation-Instruction/Linux/MBsysPad.html) and [MBsysC](http://robotran-doc.git-page.immc.ucl.ac.be/Installation-Instruction/Linux/MBsysC.html) in order to use the C/C++ version of Robotran. 

To understand the multibody system dynamics used here you can read the [Robotran Basics](https://www.robotran.be/images/files/Robotran_basics.pdf). 

Here also in order to have a better understanding and management of Robotran you can follow the [Getting Started Tutorial in C/C++](http://robotran-doc.git-page.immc.ucl.ac.be/TutorialModellingFeatures/output_tuto/c-code/). If you want to go further you can also follow the [Equilibirum and Modal (not used here) module of Robotran Tutorial](http://robotran-doc.git-page.immc.ucl.ac.be/TutorialModellingFeatures/Modules/output_mod/c-code/). 

This package also use the real-time features of Robotran so please follow [this tutorial](https://www.robotran.be/tutorial/realtime/).

Finally, in order to link Robotran and ROS, we need to use multithreading. Update the package index `$ sudo apt-get update` and install the pthread library `$ sudo apt-get install libpthread-stubs0-dev`.
  

## Compilation and run instructions
1) Download this package in your *catkin_ws/src* folder.
2) Go to the CMakeLists of each Robotran project  (for example : */home/user_name/catkin_ws/src/ros_rob/src/Car/workR/CMakeLists.txt*) and set the correct way to the MBsysC folder with */home/user_name/.robotran/mbsysc-dev/MBsysC/*.
3) Go to the *catkin_ws* folder et compile/build it with: `~/catkin_ws$ catkin_make`
4) Run `$ roscore` on a terminal. 
5) Run `$ rosrun ros_rob exe_Car` on a new terminal. It starts the ROS node wich run the real-time simulation in Robotran and publish the torque on a topic for a *listener node*. The node closes automatically at the end of the simulation.
6) Close *roscore* by pressing ctrl+c in the terminal when the simulation is finished. 


## Add your own Robotran Project
The example here is made with a Robotran Project wich simulates the dynamics of a car. But you can also add your own Robotran project (other type of vehicles, other multibody system, ...). 

This tutorial assumes you followed the **Installation and configuration of ROS** and **Installation and configuration of Robotran** sections.

1) Download this package in your *catkin_ws/src* folder.

2) Copy all the files and folders of your Robotran project (userfctR, workR, ect) without *build* folder in *ros_rob/src/name_of_your_project*.

3) Then add the CMakeList of Robotran to the one of ROS by modifying this line in the ROS CMakeList 

      `add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/name_of_your_project/workR)`

4) Include the ROS and Pthread libraries in the Robotran CMakeList 

         include_directories(/opt/ros/melodic/include /opt/ros/melodic/lib)

         target_link_libraries(${Executable} -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -lroscpp -lrostime -lrosconsole -lroscpp_serialization)
         
         target_link_libraries(${Executable} -lpthread)

5) Change the extension of the Robotran *main.c* by *main.cpp*. Change the content of the main with the template [*.main_robotran_template.cpp*](https://git.immc.ucl.ac.be/huensf/ros_rob/-/blob/master/.main_robotran_template.cpp) and follow the change instructions inside the template. 

6) As you can see at point 5. and 6. we need to use a user model structure from Robotran. Follow [the user model tutorial for structures](http://robotran-doc.git-page.immc.ucl.ac.be/usermodelstructurec/) in order to initialize it in *MBSysPad* and create an header file (also see [*thread_struct.h*](https://git.immc.ucl.ac.be/huensf/ros_rob/-/blob/master/src/Car/userfctR/thread_struct.h) for an example of declaration of the structure). This structure contains pointer to function like *give_torque_access* wich allows ROS publisher and suscriber to have access to *mbs_data* without segmentation fault cause of the multithreading. After that you can give the access to the ROS publisher and suscriber in all the user function with :
 
         (*mbs_data->user_model->thread.thread_struct->pointeur_<name_of_access_function>)();
         
 7) **Note** : the main of Robotran is now in C++ but the user functions that you will use are usually in C and are called by this C++ function. You can have a look on [Name Mangling and extern “C” in C++](https://www.geeksforgeeks.org/extern-c-in-c/) to ensure that the C++ compiler behaves like a C compiler for thess functions.  

 8) Go to the catkin_ws folder et compile/build it with :`~/catkin_ws$ catkin_make`
 9) Run `$ roscore` on a terminal. 
10) Run `$ rosrun ros_rob exe_your_project_name` on a new terminal. It starts the ROS node wich run the real-time simulation in Robotran and publish the torque on a topic for the *listener* node. The node closes automatically at the end of the simulation. 
11) Close  *roscore* by pressing ctrl+c in the terminal.


### Deal with multiple Robotran projects 

If you need to deal with several Robotran projects, just follow the above instructions one more time with two exeptions :

  * At the point 3. you can have only one link with a Robotran CMakeLists (only one *add_subdirectory*). 
  * At the point 8., before compile/build your workspace you need to delete the *ros_rob* folder in the *build* folder. 

This means that you can compile and build only one Robotran project at a time. But once the new ROS executable is created (here the *exe_your_project_name*) you can run which you want. 

### Use your own ROS message

The ROS messages here are used to vehicle data for the torque, position and velocity of a steerwheel wich are floating point values. It could be usefull to create your own ROS messages in order to deal with the values provided and needed by your Robotran project. You can consult this [tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) and this [ROS wiki](http://wiki.ros.org/msg) for more informations. 




