SteerForce Converter for Morse Simulation
==================

This pacakge provides an option to test the new ROS_CONTROL in the simulation. If the appropriate parameter is set to true then the package would subscribe to /ackerman_cmd to calcualte the force and angle.If this is set to false /cmd_vel will be used directly to compute the actuator commands.

# Usage
1. Install simple_pid python library - `pip install simple_pid`
2. configure required parameters in params yaml file.
3. Modify morse default.py to accept commands from /morse_steerforce_cmd
4. Add launch file to req launch file to start.  

PS: Requries odometry/filtered data published by ekf_se node.

