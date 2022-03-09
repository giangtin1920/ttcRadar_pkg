#include "ttcRadar_Cfg.h"

#define DataPort_EN

ttcRAdarObj ttcRadarObj;
ros::Publisher ttcRadar_pub;
ttcRadar_pkg::ttcRadar_msg ttcRadar_output_msg;

void timer_uart_Callback(const ros::TimerEvent& )
{
    if(ttcRadarObj.ser_Data_Port.available())
    {
        uint16_t dataLen = 0;
        dataLen = ttcRadarObj.ser_Data_Port.available();
        std_msgs::UInt8MultiArray raw_data;
        ttcRadarObj.ser_Data_Port.read(raw_data.data, ttcRadarObj.ser_Data_Port.available());
        ROS_INFO("Read: %u byte ---------------------", dataLen);

        // Process the raw_data
        if (true == ttcRadarObj.data_handler(raw_data, dataLen))
        {
            switch (modeRadar)
            {
                case ENABLE_RADAR_TTC:
                {

                }
                break;

                case ENABLE_RADAR_MPC:
                {
                    // Send ros message
                    ttcRadar_output_msg.msg_counter = ttcRadarObj.Output.msg_counter;
                    ttcRadar_output_msg.isObject = ttcRadarObj.Output.isObject;
                    ttcRadar_output_msg.distance = ttcRadarObj.Output.distance;
                    ttcRadar_pub.publish(ttcRadar_output_msg);
                    ROS_INFO("Public message ok (MPC)");

                }
                break;

                default:
                break;
            }
            
        }
    }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "ttcRadar");
    ros::NodeHandle n;
    ttcRadar_pub = n.advertise<ttcRadar_pkg::ttcRadar_msg>("ttcRadar_Data", 1000);
    
    // Timer to receive data from Radar
    ros::Timer timer_uart = n.createTimer(ros::Duration(0.05), timer_uart_Callback);

    // Connect to COM port of Radar 
    if (true == ttcRadarObj.init_cfg_port())
    {
        #ifdef DataPort_EN
        if (true == ttcRadarObj.init_data_port())
        {

        }
        else
        {
            return -1;
        }
        #endif
    }
    else
    {
        return -1;
    }
    
    ttcRadarObj.start_radar();

    // While loop do nothing, data received by interrupt
    while(ros::ok())
    {
        ros::spinOnce();
    }
    
}
