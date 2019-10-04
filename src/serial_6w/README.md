Note that the default protocol for six wheeled car was not usable for our cases. 
There are two protocols we have created.

One of them to send data to serial_communicator node.

AT SixWheelCommand.msg
if you set controltype to 0 you can set rigth and left speed seperately.
if you set this to 1 the car uses its own control method where it accepts only speed and angle as input. It gets speed and multiplies it with angle percetage and choses the other sides speed.
if you set it to 2 you can controll tyres seperately.

Between MCU and ROS communicatios we are only using bytes so;
The introduction and ending of the message is always same. And you can see an example byte array
to controll the car.
1 	--> Start Byte (1)
2	-->MCU Address (255)
3 	--> Message Length
4	--> Controll Type 
5 	--> Info Bytes
6	--> Info Bytes
7 	--> Info Bytes
8 	--> Info Bytes
9	--> End Byte (4)


We used three controll types
1) The one we controll left and right seperately

1 	--> Start Byte(1)
2	--> MCU Address(255)
3 	--> Message Length (7)
4	--> Direction Select (52,53,54 or 55)
5 	--> Info Bytes (Right Speed)
6	--> Info Bytes (Left Speed)
7	--> End Byte (4)

To choose Control Type byte
52: Both Right and Left side goes forward
53: Right Side backard left side forward
54: Right Side forward left side backward
55: Both Right and Left side goes bacward


2) Default Controller 

1 	--> Start Byte(1)
2	--> MCU Address(255)
3 	--> Message Length (8)
4	--> Controll Type (51)
5 	--> Direction Select (1,2,3 or 4)
6	--> Info Bytes (speed)
7	--> Info Bytes (angle)
8	--> End Byte (4)

1: Go forward Turn clocwise
2: Go Forward Turn counter clockwise
3: Go backward Turn clocwise
4: Go bacward Turn counter clockwise

3) Individual motor controll
1 	--> Start Byte(1)
2	--> MCU Address(255)
3 	--> Message Length (7)
4	--> Controll Type (motor number+9)
5 	--> Info Bytes (speed)
6	--> Info Bytes (direction)(1 forward or 2 backward)
8	--> End Byte (4)

