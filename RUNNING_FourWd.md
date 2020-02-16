# How to Run

## Four Wheeled (4WD) Platform

### Connecting to the Jetson PC

Connect to the *CARS* wifi with password *Test1234*
Then, ssh terminals to Jetson PC:  

`ssh jetson2@192.168.1.188`
password for the jetson2: *jetsongtarc*

## Project Structure

### System is compromised of four basic components.

- Odometry Fusion package which initializes sensors and starts SLAM algorithms. 
- SLAM Package 
- Navigation Stack - Consists
	* Local planner which calculates cmd_vel given odometry and path data.
 	* Global planner which calculates the path from start to the end.
- Morse Simulation Engine.	 
- Motor Controller Interface which takes cmd_vel message and translates it into motor commands.

### bear_car_launch Package
Main Package consisting of all necessary launch files and config files to parameterize the various packages used in the software stack. 

## Running the project

#### Running Autonomous Navigation (simulation)

- To run everything at once use the bash script
	- `source run_all`
- To run manually

	- `source devel/setup.bash`
	- `roslaunch bear_car_launch autonomous_navigation_fourwd_sim.launch`
	- To visualise
	  * `rosrun rviz rviz -d `rospack find bear_car_launch`/rviz_cfg/rviz_navigation_cc.rviz`

To enable different functionalities and further details please refer `bear_car_launch` readme files.


#### Running Autonomous Navigation (Real Car)

For the Real Car, you can either run the whole stack on the car by 
- `source autonomous_navigation_fourwd.launch`  ** Remember to uncomment the required lines in the launch file

Or it is possibel to run only low level packages on the car and run SLAM and navigation stack on the HOST PC
The following commands enable it.
* On Jetson 
	- `source devel/setup.bash`
	- `roslaunch bear_car_launch autonomous_navigation_fourwd.launch`.

* On HOST PC - 
	- `source scripts/remoteClientNetwork.sh jetson2`
	- `source devel/setup.bash`
	- `source Navigation_SLAM.launch`

### For more information refer bear_car_launch package README 


