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
