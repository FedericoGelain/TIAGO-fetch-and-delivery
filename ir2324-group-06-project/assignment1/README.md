GROUP 06 <br />
Alberto Dorizza, alberto.dorizza@studenti.unipd.it <br />
Dario Mameli, dario.mameli@studenti.unipd.it <br />
Federico Gelain, federico.gelain@studenti.unipd.it

<br />

To correctly open the world simulation in Gazebo and activate the navigation to move using move_base the command to type are:
<ol>
    <li> <b><i> roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library </b></i> </li>
    <li> <b><i> roslaunch tiago_iaslab_simulation navigation.launch </b></i> </li>
</ol>

As for the communication between client and server, after the commands above are run, there are two possible settings:
<ul>
    <li> to activate the motion control law, <b><i>  rosrun assignment1 assignment1_client 11 0 -45 1 </b></i>, where the last command line argument is the flag which activates it </li>
    <li> to run the simulation using only move_base, type either <b><i> rosrun assignment1 assignment1_client 11 0 -45 0 </b></i> or <b><i> rosrun assignment1 assignment1_client 11 0 -45 </b></i> </li>
</ul>
Where 11 and 0 are respectively the x and y coordinates of pose_B, -45 is the orientation in degrees (clockwise rotation along the z axis) <br />
To activate the server, the command is for both cases <b><i> rosrun assignment1 assignment1_server </b></i>
