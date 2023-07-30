/*  Copyright @ JUN YOU

    This generates a weather node with functions as:
    1. read data source from Peetbrother weather station 800 RS232 i/o port
    2. publish relavant environemental ROS topics
   
*/

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <std_msgs/UInt16.h>
#include <ma_dt_quadrotor/Wind.h>



int hexToDecimal(const std::string& hexValue) {
    int decimalValue = 0;
    for (char c : hexValue) {
        decimalValue = decimalValue * 16 + (isdigit(c) ? c - '0' : c - 'A' + 10);
    }
    return decimalValue;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "env_node");
    ros::NodeHandle nh;
    ros::Rate rate(2.0);

    // create sensor msg
    ma_dt_quadrotor::Wind current_wind;

    // create topic handle
    ros::Publisher wind_pub = nh.advertise<ma_dt_quadrotor::Wind>("/environment/wind", 10);



    // int serialPort = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

    // if (serialPort == -1) {
    //     ROS_ERROR("Error opening the serial port.");
    //     return 1;
    // }

    // struct termios tty;
    // tcgetattr(serialPort, &tty);

    // tty.c_cflag &= ~PARENB;  // Disable parity bit
    // tty.c_cflag &= ~CSTOPB;  // Set stop bit to 1
    // tty.c_cflag &= ~CSIZE;   // Clear character size mask
    // tty.c_cflag |= CS8;      // Set data bits to 8

    // tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    // tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    // tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control

    // tty.c_lflag = 0; // No local flags
    // tty.c_oflag = 0; // No output processing
 
    // tty.c_cc[VTIME] = 0; // Wait for up to 1 second, returning as soon as any data is received
    // tty.c_cc[VMIN] = 0;   // Return immediately if no data is available

    // cfsetispeed(&tty, B2400); // Set input baud rate
    // cfsetospeed(&tty, B2400); // Set output baud rate

    // tcsetattr(serialPort, TCSANOW, &tty);





    while(ros::ok()){

        // // READ WEATHER I/O DATA AND TRUNCATE INTO current_wind
        // char buffer[256];
        // int bytesRead = read(serialPort, buffer, sizeof(buffer));

        // if (bytesRead > 0) {
        //     std::string data(buffer, bytesRead);
        //     std::string hexValues = "";
       
        //     // Extract the first 4 hexadecimal numbers
        //     int count = 0;
        //     bool get_info = false;
        //     for (char c : data) {
        //         if (c == '!') {
        //             get_info = true;
        //         }
                
        //         if (std::isxdigit(c) && get_info) {
        //             hexValues += c;
        //             count++;
        //         }

        //         if (count == 4) {
        //             break;
        //         }
        //     }

        //     // Convert the hexadecimal value to decimal
        //     int decimalValue = hexToDecimal(hexValues);



        //     // Extract the 4th to 8th decimal numbers
        //     std::string hexValues_direction = "";
        //     get_info = false;

        //     count = 0;
        //     for (char c : data) {
        //         if (c == '!') {
        //             get_info = true;
        //         }
                
        //         if (std::isxdigit(c) && count<5 && get_info) {
        //             count++;
        //         }else if(std::isxdigit(c) && count>4){
        //             hexValues_direction += c;
        //             count++;
        //         }

        //         if (count == 8) {
        //             break;
        //         }
        //     }

        //     // Convert the extracted decimal values to decimal
        //     int decimalValue_direction = hexToDecimal(hexValues_direction);

            // int wind_cali = 0;

            // float wind_speed = decimalValue;
            // wind_speed = wind_speed/36;
            // int wind_direction = decimalValue_direction*360/255 + wind_cali;
            // float wind_direction_rad = wind_direction*M_PI/180;

            // current_wind.wind_x =  wind_speed * std::cos(wind_direction_rad);
            // current_wind.wind_y = - wind_speed * std::sin(wind_direction_rad);
            // current_wind.wind_z = 0;
            // // ROS_INFO_STREAM("\n WIND SPEED original: "<< hexValues << " | WIND DIRECTION: original " << hexValues_direction);

            // ONLY FOR TEST
            current_wind.wind_x = 0;
            current_wind.wind_y = 2;
            current_wind.wind_z = 0;

            // ROS_INFO_STREAM("\nWIND SPEED: "<< wind_speed << " | WIND DIRECTION: " << wind_direction << "\n");
            // ROS_INFO_STREAM("\nWIND VECTOR: \n"<< current_wind);

            wind_pub.publish(current_wind);
        // }



        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}
