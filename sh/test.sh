roslaunch mavros px4.launch & sleep 3;
roslaunch vrpn_client_ros sample.launch server:=192.168.31.107 & sleep 3;
roslaunch launch/test.launch 
