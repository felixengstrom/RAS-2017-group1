# Group 1 project:
Felix Engström
Thibault Jean Yves Laval
Dan Lillrank
Albina Shilo
Sarangi Veeramani Lekamani

#Building the project:
catkin_make
#Build individual package:
catkin_make -DCATKIN_WHITELIST_PACKAGES="<package name>"
  
#Launching the executables
roslaunch <package name> <executable name>
roslaunch ras_project_launch robot_launch.launch 
roslaunch ras_project_tf_listener tf.launch
