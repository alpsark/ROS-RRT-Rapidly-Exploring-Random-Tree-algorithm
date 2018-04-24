# RRT-Rapidly-Exploring-Random-Tree-algorithm

cd ros_ws  
sudo apt-get install ros-kinetic-joy  
run "catkin_make" make sure there are no errors.  
edit ~/.bashrc, add "source /home/..../rosws/devel/setup.bash" to the end of the .bashrc file.  
close the terminal and reopen the terminal   
run roscore  
run vrep, open scenes/assignment8.ttt  
. ~/ros-ws/devel/setup.bash  

**To go to pose:** we need 2 consoles  
rviz  
roslaunch myrrt rrtplanning.launch    

then we can move the robot with arrow keys  


**To move base:** (move like the assignment 7 gmapping and navigation)
rviz
roslaunch nav_stack_tuning move_base.launch    
roslaunch nav_stack_tuning amcl.launch    
  
