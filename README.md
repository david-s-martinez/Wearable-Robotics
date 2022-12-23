# ROS
<br />
Run in different terminals:
<pre>
roscore
</pre>
<pre>  
rosrun rosserial_python serial_node.py /dev/ttyACM0 (or whatever port)
</pre>
<pre>  
cd exo_ws
</pre> 
<pre> 
source ./devel/setup.bash
</pre> 
<pre> 
rosrun servo_control publisher.py
</pre> 
<br/>

<br/>
To send angle run (angle 30-120):

<pre> 
rostopic pub servo std_msgs/Float32 120.0 
</pre> 
<br/>
launch cpp exo control :

<pre> 
roslaunch exo_control exo_control.launch
</pre> 
<br/>
check skin :

<pre> 
skineventscmd
</pre> 
<pre> 
ftdi v2
</pre> 
<pre> 
c
</pre> 
<br/>
<br/>
configure skin :

<pre> 
roslaunch tum_ics_skin_full_config full_config.launch
</pre> 

<br/>

launch cpp skin control :
<pre>
roslaunch tum_ics_skin_driver_events skin_driver_ftdi.launch FTDI_SERIAL:=FT601ZA5
</pre>
<pre> 
roslaunch skin_control skin_control.launch
</pre>
<pre> 
rostopic echo /patch1/data[0] 
</pre>
<pre> 
rosmsg show tum_ics_skin_msgs/SkinCellDataArray
</pre>
<br/>
