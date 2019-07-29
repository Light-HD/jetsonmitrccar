#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "serial_6w/SixWheelCommand.h"
#include "serial_6w/SixWheelInfo.h"
// C library headers
#include <stdio.h>
#include <string.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
//TODO: Bacward direction error

class Translator{
  public:
  Translator()
  { 
    ROS_INFO("Initializing");
    n = ros::NodeHandle("~");
    motor_cmd_sub = n.subscribe("motor_commans", 1000,&Translator::motorcmdCallback,this);
    motor_info_pub = n.advertise<serial_6w::SixWheelInfo>("motor_controler_info", 1000);   
    //motor_info_timer = n.createTimer(ros::Duration(0.1),&Translator::motorinfoCallback,this); //every 100ms for now
    motor_cmd_timer = n.createTimer(ros::Duration(0.2),&Translator::motorcmdCallback,this); //every 100ms for now
    bytes_to_send = new unsigned char[10];
    serial_buffer= new unsigned char[20];

    //////////////////
    //Initilise USB//
    /////////////////

  serial_port = open("/dev/ttyUSB0", O_RDWR);
  // Check for errors
  if (serial_port < 0) {
  ROS_INFO("Error %i from open: %s\n", errno, strerror(errno));
  }
  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
  ROS_ERROR("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }
  tty.c_cflag &= ~PARENB;    // Clear parity bit, disabling parity (most common)
  // tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  // tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  //tty.c_cflag |= CS5; // 5 bits per byte
  //tty.c_cflag |= CS6; // 6 bits per byte
  //tty.c_cflag |= CS7; // 7 bits per byte
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  // tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  // To enable followings just change '&' with '|'
  tty.c_lflag &= ~ICANON; //(Dis)Enable Canonical(ie: processes when new line recieved) 
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;
  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B9600);   //B0,  B50,  B75,  B110,  B134,  B150,  B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
  cfsetospeed(&tty, B9600);
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) { 
  ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
   ROS_INFO("Initialised");
 }

void motorcmdCallback(const serial_6w::SixWheelCommand::ConstPtr& msg)  
{
ROS_INFO("Motor Command Call Back");

  if(msg->controltype)
  {
    bytes_to_send[0]=1;
    bytes_to_send[1]=255;
    bytes_to_send[2]=8;
    bytes_to_send[3]=51;
    if((msg->linearspeed)>0)
    {
      if((msg->angle)>0)
        {
          bytes_to_send[4]=2;
        }
      else
        {
          bytes_to_send[4]=1;
        }
    }
    else if((msg->linearspeed)==0)
    {
      bytes_to_send[4]=0;
    }
    else
    {
      if((msg->angle)>0)
      {
        bytes_to_send[4]=4; //we will see :)//TODO: Bacward direction error
      }
      else
      {
        bytes_to_send[4]=3; //we will see :)//TODO: Bacward direction error
      } 
    }

    bytes_to_send[5] = abs(msg->linearspeed);
    bytes_to_send[6] = abs(msg->angle);
    bytes_to_send[7] = 4; //Stop Byte
    bytes_to_send[8] = '\0';
  }
  else
  {
    bytes_to_send[0] = 1;
    bytes_to_send[1] = 255;
    bytes_to_send[2] = 7;
    bytes_to_send[3] = (msg->motor_number + 9);
    bytes_to_send[4] = abs(msg->individual_motors_speed);

    if((msg->individual_motors_speed)>0)
    {
      bytes_to_send[5]=2; 
    }
    else if ((msg->individual_motors_speed)<0)
    {
      bytes_to_send[5]=1; 
    }
    else
    {
      bytes_to_send[5]=0; 
    }

    bytes_to_send[6]=4;
    bytes_to_send[7] = '\0';

  }

  write(serial_port, (char*)bytes_to_send , sizeof(char)*bytes_to_send[2]);
  }

  void motorcmdCallback(const ros::TimerEvent&)
  {
    ROS_INFO("OKUYOM");
    write(serial_port, (char*)bytes_to_send , sizeof(char)*bytes_to_send[2]);
    int num_bytes = read(serial_port, serial_buffer, sizeof(char)*30);
    if (num_bytes < 0) {
    ROS_INFO("Error reading: %s", strerror(errno));
  }

  u_int8_t State = 0;
  // This variable counts the received bytes
  u_int8_t Bytes_Number = 0;
  // This variable contains the expected message size  
  u_int8_t BytesToReceive = 0;
  // This variable contains the information size  
  u_int8_t InformationSize = 0;
  // This vabiable contains the ControlByte
  u_int8_t RecievedControlByte = 0;
  // This variable keep the current number of information byte
  u_int8_t InformationByteCounter = 0;

  while(State<5)
  {		
    // State 0 : IDLE
		if(State==0)  
    { // If the StartByte received
      ROS_INFO("STATE1");
	  	if (serial_buffer[Bytes_Number] == 1 ) //Protocol Start Byte
			{
        State++; //Pass the Stage 1
				Bytes_Number++; // Received = 0 + 1 = 1
			}
			else
			{
        ROS_ERROR("Start Byte is not true!");
				State = 0;
        break;
			}			
		}	

		// State 1 : Check Adress
		  if(State==1)
      {
        ROS_INFO("STATE2");
        // If the ReceivedByte is My_Adress
				if (serial_buffer[Bytes_Number] == 255) //My Adress
				{
				  Bytes_Number++; // Received 1 + 1 = 2
				  State++; // Go to the next state
				}
				else
				{
          ROS_ERROR("Adress is not ture!!");
          // Otherwise go to the first statement 
				  State = 0;
          break;
				}
				}					
		// State 2: Check message size and calculate Information size
		  if(State==2) 
      {
        ROS_INFO("STATE3");
				// The RecievedByte in this state contains the size of the message
				BytesToReceive = serial_buffer[Bytes_Number];
				// Calcule the size of information bytes : That is BytesToReceive minus startbyte, adressbyte, lenghtbyte, controlbyte and stopbyte = 5
				InformationSize = (BytesToReceive - 5);
				// Received 2+ 1=3
				Bytes_Number++;
				// Go to the next state
				State++;
					// Reset stopwatch
					//setStopwatch1(0);
			}					
	
		// State 3: Check control byte
		if(State==3) 
      {
        ROS_INFO("STATE4");
					// Put the ReceivedByte in RecievedControlByte
					RecievedControlByte = serial_buffer[Bytes_Number];
					// Received 3+ 1=4
					Bytes_Number++;
					// Go to the next state
					State++;
			}					

		// State 4: 
		if(State==4)   
      {
        ROS_INFO("STATE5");

        info_message.linearspeed = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor1_speed = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor2_speed = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor3_speed = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor4_speed = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor5_speed = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor6_speed = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor1_current = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor2_current = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor3_current = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor4_current = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor5_current = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.motor6_current = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.voltage = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;
        info_message.temprature = serial_buffer[InformationByteCounter+4];
        InformationByteCounter++;

					// Fill information array with received byte(s)
					//RecievedInformationBytes[InformationByteCounter] = ReceivedByte;
					// InformationByteCounter +
					// Received + 1
					Bytes_Number=Bytes_Number+InformationByteCounter;
					// If this byte was the last information byte
					if(InformationByteCounter == InformationSize)
					  {
				  		// Go to the next state
				  		State++;
					  }
          else
            {
              break;
            }				
				}
				  
		// State 5: 
				if(State==5)  {
          ROS_INFO("STATE6");
					// Received + 1
					Bytes_Number++;
					// If the stop byte is received and the number of receiverd bytes = expexed bytes, then the hole packet is reveived
					if((serial_buffer[Bytes_Number-1]==4) & (Bytes_Number==BytesToReceive))//stop Byte
					  {
						// Process received data
						ROS_INFO("Message is in integrity!!!");
            State++;
            motor_info_pub.publish(info_message);
					  }
            else
            {
            ROS_ERROR("FAILED");
            break;
            }
				  }			  				
  
    } 
 }
    
  private:
  ros::NodeHandle n;   
  ros::Subscriber motor_cmd_sub;
  ros::Publisher motor_info_pub;
  ros::Timer motor_cmd_timer;
  int serial_port;
  u_int8_t *bytes_to_send;
  u_int8_t *serial_buffer;
  serial_6w::SixWheelInfo info_message;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_communicator"); 
  Translator serial;
  
  ros::spin();
  return 0;
}