# software_integration

Integration repository for all relevant software pieces/submodules.
This project normally runs on Nvidia jetsonTX2 PCs on the cars (already installed).
If you would like to run some of the packages on your PC, please refer to the Build section.


##Questions/TODOs in/about submodules/directories

1. buildRealsenseTX2 did we change anything here? Why not a submodule if not?
2. FourWheelCarConfigurations(VESC) needs README
3. SixwheelFırmware needs README
4. arduino has TODO in Readme
5. jetsonTX2Power TODOS in Readme
6. Where is the 4WD simulation environment?
7. Morse basicspeed.py is missing code docu, currently mostly boilerplate docu.
8. TODO add the end in all of your code either document why codes are commented or remove commented code.
9. TODO all of your packages require a README that exlains what it is
10. low_level_speed_controller fails during compilation.


## Structure

**TODO Describe repo structure, all included packages have to be mentioned and for what are they responsible, how should/can they be used briefly, more detailed information should be referenced in a subreadme within the corresponding package/module. Here, you should also make clear what is just a git submodule from someone else and what you/we have been developed**


## Build

**TODO Describe cloning (recursive) + dependency resolving/installs + build**

**TODO This should also reflect the different environments such as Arduino, ROS, Linux Kernel modules, AVR for 6WD motorcontoller ...**

**TODO here we could also differentiate between what has to be build on the car and what on the developer machine? SHOULD we create two integration repositories for this?** << This would also save a lot of space on the jetson because there we dont need simulation etc.

**TODO is this enough for resolving dependencies?**

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Run

### Running Four Wheel
** TODO Describe how to run the code, potentially explaining different launch files for different purposes**

** TODO Different setups are, for instance remote controlled driving, move_base-based driving, different simulation engines.

### Running Six Wheel
blabla


## Other Documentations

** TODO add the links of the other README files for each package
**TODO if appropriate link READMEs of subdirectories/modules below**


## Sensors Setup

In our platforms we are using following sensors

- Wheel Encoder:

  Six Wheeled platform has its own wheel encoders for each wheels but they are only one phase encoders. Therefore they can't sense rotation direction, we solved this issue at odometry node assuming wheels never slides and rotates in commanded direction. Four wheeled system uses ERPM readings from BLDC motor. We also integrated an additional wheel encoder on it but not integrated it to ROS environment.

- Laser Scanner:

  Both cars have their RPLidar A3 laser scanner.

- IMU (Integrated on Camera):

  Intel D435i cameras have integrated IMU sensors on them.

- Camera:

  For  many further applications we also have a Depth Camera on  platforms.


## Open Issues
 1. For VESC We applied Encoder that we can read it from VESC Firmware. To more accurate speed readings, ıt should also be implemented for ROS. Note that Vesc only reads
 encoder as rotor position. Therefore, two there exists two steps add position reading to VESC_Driver node Convert it into speed.

 2. Motor controller for six wheeled platform' motor controller node (serial_6w) needs a launch file to easily change parameters especially USB device name.

 3. 6 Wheeled platform's encoder readings is not that reliable. For now we are using rf2o node for odometry. Wheel odometry data my be improved lasoftware_integration
     Integration repository for all relevant software pieces/submodules.
     
 4. 6 Wheeled platform's one motor has damaged. It should be changed. Please check the issue .

 5. This project normally runs on Nvidia jetsonTX2 PCs on the cars (already installed).
     If you would like to run some of the packages on your PC, please refer to the Build section.

 6. PID performances can be improved to see the interface please loo interface section.

     

     

     ##Questions/TODOs in/about submodules/directories

     **TODO Describe repo structure, all included packages have to be mentioned and for what are they responsible, how should/can they be used briefly, more detailed information should be referenced in a subreadme within the corresponding package/module. Here, you should also make clear what is just a git submodule from someone else and what you/we have been developed**

     Build
     TODO Describe cloning (recursive) + dependency resolving/installs + build

     TODO This should also reflect the different environments such as Arduino, ROS, Linux Kernel modules, AVR for 6WD motorcontoller ...

     TODO here we could also differentiate between what has to be build on the car and what on the developer machine? SHOULD we create two integration repositories for this? << This would also save a lot of space on the jetson because there we dont need simulation etc.

     TODO is this enough for resolving dependencies?

     ￼
     rosdep install --from-paths src --ignore-src -r -y
     Run
     Running Four Wheel
     ** TODO Describe how to run the code, potentially explaining different launch files for different purposes**

     ** TODO Different setups are, for instance remote controlled driving, move_base-based driving, different simulation engines.

     Running Six Wheel
     blabla

     Other Documentations
     ** TODO add the links of the other README files for each package
     TODO if appropriate link READMEs of subdirectories/modules below

     

     

     Open Issues

      7.

      8.

     #FAQ

     **TODO if you have collect solutions for common pitfalls (maybe things you experienced yourself)**ter.

 7. 6 Wheeed platform's one motor has demaged. I shoul be changed. Please check the issue .

 8. We do not have any visual odometry. General odometry performance and suggestions.

 9. 

 7.

 8.

#FAQ

**TODO if you have collect solutions for common pitfalls (maybe things you experienced yourself)**