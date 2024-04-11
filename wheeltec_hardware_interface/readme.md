
Start roscore

`roscore`

Start microcontroller hardware node

`rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600`

Start wheeltec hardware interface 

`roslaunch wheeltec_hardware_interface wheeltec_hardware_bringup.launch`

Start moveit planning 

`roslaunch wheeltec_gripper_moveit_config demo.launch`

