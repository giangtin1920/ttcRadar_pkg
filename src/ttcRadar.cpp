#include "ttcRadar_Cfg.h"

#define DataPort_EN

ttcRAdarObj ttcRadarObj;
ros::Publisher ttcRadar_pub;
ttcRadar_pkg::ttcRadar_msg ttcRadar_output_msg;

void timer_uart_Callback(const ros::TimerEvent& )
{
    uint16_t dataLen = ttcRadarObj.ser_Data_Port.available();
    if (!dataLen) return;

    std_msgs::UInt8MultiArray raw_data;
    ttcRadarObj.ser_Data_Port.read(raw_data.data, dataLen);
    ROS_INFO("Read: %u byte -----------------------------,", dataLen);

    // Processing the raw_data
    if (!ttcRadarObj.data_handler(raw_data, dataLen)) return;
    switch (modeRadar)
    {
        case ENABLE_RADAR_TTC:
        {
            for (auto i = 0; i < ttcRadarObj.Output.numTrackedObj; i++)
            {
                ttcRadar_output_msg.msg_counter = ttcRadarObj.Output.msg_counter;
                ttcRadar_output_msg.isObject = ttcRadarObj.Output.isObject[i];
                ttcRadar_output_msg.distance = ttcRadarObj.Output.distance[i];
                ttcRadar_pub.publish(ttcRadar_output_msg);
                ROS_INFO("Public message ok (TTC) \r\n");
            }
        }
        break;

        case ENABLE_RADAR_MPC:
        {
            // Send ros message
            ttcRadar_output_msg.msg_counter = ttcRadarObj.Output.msg_counter;
            ttcRadar_output_msg.isObject = ttcRadarObj.Output.isObject[0];
            ttcRadar_output_msg.distance = ttcRadarObj.Output.distance[0];
            ttcRadar_pub.publish(ttcRadar_output_msg);
            ROS_INFO("Public message ok (MPC) \r\n");
        }
        break;

        default:
        break;
    }
        
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "ttcRadar");
    ros::NodeHandle n;
    ttcRadar_pub = n.advertise<ttcRadar_pkg::ttcRadar_msg>("ttcRadar_Data", 1000);
    
    // Timer to receive data from Radar
    ros::Timer timer_uart = n.createTimer(ros::Duration(0.05), timer_uart_Callback);

    // Connect to COM port of Radar 
    if (!ttcRadarObj.init_cfg_port()) return -1;
    
    #ifdef DataPort_EN
    if (!ttcRadarObj.init_data_port()) return -1;
    // {
    // }
    #endif
    
    ttcRadarObj.start_radar();

    // While loop do nothing, data received by interrupt
    while(ros::ok())
    {
        ros::spinOnce();
    }   
}
