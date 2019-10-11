This Codes proved the communication between Main MCU on Motor controller and Nvidia TX2.

Note that the default protocol for six wheeled car was not usable for our cases. 
There are two protocols we have created.

One of them to send data to serial_communicator node.

At SixWheelCommand.msg if you set control type to 0 you can set right and left side speed separately.
if you set this to 1 the car uses its own control method where it accepts only speed and angle as input. If it gets an angle it slows down the tires at the turning direction according to the formula: 

Speed = Given_Speed-(Given_Speed*Angle/125)  if you set it to 2 you can control tires separately.

Between MCU and ROS communications we are only using bytes so;

The introduction and ending of the message is always same. And you can see an example byte array to control the car.

1. 	--> Start Byte (1)
2.	-->MCU Address (255)
3. 	--> Message Length
4.	--> Control Type 
5. 	--> Info Bytes
6.	--> Info Bytes
7. 	--> Info Bytes
8. 	--> Info Bytes
9.	--> End Byte (4)

***We used three control types***

##### 1) The one we control left and right separately

1. 	--> Start Byte(1)
2.	--> MCU Address(255)
3. 	--> Message Length (7)
4.	--> Direction Select (52,53,54 or 55)
5. 	--> Info Bytes (Right Speed)
6.	--> Info Bytes (Left Speed)
7.	--> End Byte (4)

To choose Control Type byte
* 52: Both Right and Left side goes forward
* 53: Right Side backward left side forward
* 54: Right Side forward left side backward
* 55: Both Right and Left side goes backward

##### 2) Default Controller 

It Gets a speed and sends it to them. If it gets an angle it slows down the tires at the turning direction according to the formula: 

Speed = Given_Speed-(Given_Speed*Angle/125)  

1. 	--> Start Byte(1)
2.	--> MCU Address(255)
3. 	--> Message Length (8)
4.	--> Control Type (51)
5. 	--> Direction Select (1,2,3 or 4)
6.	--> Info Bytes (speed)
7.	--> Info Bytes (angle)
8.	--> End Byte (4)

* 1: Go Forward Turn clockwise
* 2: Go Forward Turn counter clockwise
* 3: Go Backward Turn clockwise
* 4: Go Backward Turn counter clockwise

##### 3) Individual motor control

1. 	--> Start Byte(1)
2.	--> MCU Address(255)
3. 	--> Message Length (7)
4.	--> Control Type (motor number+9)
5. 	--> Info Bytes (speed)
6.	--> Info Bytes (direction)(1 forward or 2 backward)
8.	--> End Byte (4)