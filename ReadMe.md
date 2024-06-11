# Project containing a scripts for VR manipulation of a Flexiv robot

The only important file of this project is the `src/flexiv_speed_control.cpp` file. It connects the flexiv robot to ROS and listen to a topic made in Unity so that it can act as a middle layer to transfer the message to the robot. This code is in cpp as the Flexiv requires an high frequency to work in realtime, which is faster than what Python can offer.

Unfortunately, I don't have access anymore to the Flexiv robot with which this porject was made. As such, I can't properly test wether or not it is still working properly.

There was a problem with the CMakeList.txt when I first made this code but it disapear at one point and I still don't know what caused this fixed.

From how it used to work, you need the [flexiv_ros project](https://github.com/flexivrobotics/flexiv_ros) and the [flexiv_rdk](https://github.com/flexivrobotics/flexiv_rdk). I suggest to check their GitHub to before using this project.
