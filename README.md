CONVEYOR BELT

  ros2 launch conveyorbelt_gz conveyorbelt.launch.py

  ros2 run ros_gz_sim create -name RedCub -x 0.0 -y 0.0 -z 3.2 -file $(ros2 pkg prefix conveyorbelt_gz)/share/conveyorbelt_gz/sdf/aruco_box.sdf

  ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 20}

UR5 IN RVIZ

  ros2 launch ur_description view_ur.launch.py ur_type:=ur5e

UR5 IN GZ

  ros2 launch ur_simulation_gz ur_sim_control.launch.py 
