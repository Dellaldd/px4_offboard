roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" & sleep 2;
make px4_sitl_default gazebo & sleep 2;
python src/pos_control/scripts/sim/sim_circle_no_yaw.py & sleep 2;
python src/logger/scripts/logger.py;
