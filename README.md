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
To send angle run :

<pre> 
rostopic pub servo std_msgs/UInt16 1 (1 for 180 0 for 0)
</pre> 

<br/>