#include "ros/ros.h"
#include "imu/imu_cmd.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <serial/serial.h> 
# include <sensor_msgs/Imu.h>
#include <sstream>
#include "tf/transform_datatypes.h"

serial::Serial com; //Declare serial port object
ros::Publisher imuMsg_pub;


//------------------------------------------------------------------------------
// Description: Serial port sends data to the device
// Input: buf[Len]=content to be sent
// Return: Returns the number of bytes sent
//------------------------------------------------------------------------------
int UART_Write(const U8 *buf, int Len)
{
    return com.write(buf, Len); //Send serial port data
}

void imuSendCmd_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
    // com.write(msg->data);   //Send serial port data 
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");//Set encoding to prevent Chinese garbled characters
    ros::init(argc,argv,"imu_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string port_name;
    private_nh.param<std::string>("port_name", port_name, std::string("/dev/ttyUSB0"));  //Get the serial port name
    //ros::Subscriber imuSendCmd_sub = nh.subscribe("imuSendCmd", 1000, imuSendCmd_callback);
    imuMsg_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 20);

    try 
    {//Set the serial port properties and open the serial port
        com.setPort(port_name); 
        com.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        com.setTimeout(to); 
        com.open(); 
    }
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port-->" << port_name); 
        return -1; 
    }
    if(!com.isOpen()) 
    {// Serial port opening failed
        return -1; 
    } 
    ROS_INFO_STREAM("Serial Port initialized"); 

    /**
     * Set device parameters
     * @param accStill    Inertial Navigation-Station Acceleration Threshold Unitdm/s?
     * @param stillToZero Inertial navigation -static zero return speed (unit cm/s) 0: No return to zero 255: Return to zero immediately
     * @param moveToZero  Inertial Navigation -Dynamic Zero Return Speed ​​(unit: cm/s) 0: No return to zero
     * @param isCompassOn Whether to use magnetic field fusion 0: Not used 1: Used
     * @param barometerFilter The filtering level of the barometer [value 0-3], the larger the value, the more stable it is but the worse the real-time performance.
     * @param reportHz The transmission frame rate of data actively reported [value 0-250HZ], 0 means 0.5HZ
     * @param gyroFilter    Gyroscope filter coefficient [value 0-2], the larger the value, the more stable it is but the worse the real-time performance.
     * @param accFilter     Accelerometer filter coefficient [value 0-4], the larger the value, the more stable it is but the worse the real-time performance.
     * @param compassFilter Magnetometer filter coefficient [value 0-9], the larger the value, the more stable it is but the worse the real-time performance.
     * @param Cmd_ReportTag Feature subscription tag
     */

    Cmd_12(5, 255, 0,  0, 2, 60, 1, 3, 5, 0x007f); // 1.Set parameters
    Cmd_03();//2.Wake up sensor
    Cmd_19();//3.Enable active data reporting
    Cmd_05();//4.Z axis angle reset to zero


    unsigned short  data_size;
    unsigned char   tmpdata[4096] ;
    
	ros::Rate loop_rate(200);//Frequency of message publishing
    while (ros::ok())
    {//Process Imu data from the serial port
		//Number of serial port cache characters
        if(data_size = com.available())
        {//com.available(When the serial port does not have a cache, this function will wait until there is a cache before returning the number of characters.
            com.read(tmpdata, data_size);
            for(int i=0; i < data_size; i++)
            {
                Cmd_GetPkt(tmpdata[i]); // Transplantation: Fill in this function every time 1 byte of data is received. When a valid data packet is captured, it will call back and enter the Cmd_RxUnpack(U8 *buf, U8 DLen) function processing.
            }
        }
        //Process ROS information, such as subscribing to messages and calling callback functions
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }

    return 0;
}
