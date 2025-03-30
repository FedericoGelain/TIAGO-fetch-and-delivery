GROUP 06 <br />
Alberto Dorizza, alberto.dorizza@studenti.unipd.it <br />
Dario Mameli, dario.mameli@studenti.unipd.it <br />
Federico Gelain, federico.gelain@studenti.unipd.it

<br />

In order to correctly launch the package, in one terminal type the following command: 
<ul>
    <li> <b><i> roslaunch assignment2 simulation.launch </b></i> </li>
</ul>
where <b><i>simulation.launch</b></i> is the file that includes and links all necessary files, i.e: 
<ol>
	<li> The world simulation in Gazebo </li>
	<li> The <i>AprilTag</i> file launch </li>
	<li> The <i>Navigation stack</i> file launch </li>
	<li> The <i>human_node node</i> </li>
	<li> The <i>assignment1_server</i> node </li>
</ol>

Moreover, if you want to see what TIAGO camera is seeing in real-time uncomment the last line in the <b><i>simulation.launch</b></i> that includes the <i>robotCamera</i> node.
<br />

In order to run <b>rviz</b> open a new terminal and digit the following command: 
<ul>
	<li> <b><i> rosrun rviz rviz </b></i> </li>
</ul>

<br />
In two separate terminals type the following commands for running the detection and manipulation nodes of the package: 
<ul>
	<li> <b><i> rosrun assignment2 B </b></i> </li>
	<li> <b><i> rosrun assignment2 C </b></i> </li>
</ul>

<br />

In the final terminal, there are three possible ways to run the last node A:
<ul>
	<li> <b><i> rosrun assignment2 A </b></i> </li>
	<li> <b><i> rosrun assignment2 A 0 </b></i> </li>
	<li> <b><i> rosrun assignment2 A 1 </b></i> </li>
</ul>
The first two options are equivalent and will execute the entire routine without the extra points part (the cylinder coordinates are directly provided), while the third one will
use the extra point routine to determine the coordinates of the cylinders from the scan detections.


