# HiwonderController
control code for joint state to arm transmission and hard home movement


reads data from ros2 joint states node and converts it to hiwonder bus servo readable data to then publish over usb to ttl adapter at port 0
if a servo is moving opposite to rviz visualization, uncomment lines 75-78 and add in the specific servo number
