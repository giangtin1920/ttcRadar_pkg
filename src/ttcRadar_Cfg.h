#ifndef TTC_RADAR_CFG_H
#define TTC_RADAR_CFG_H

#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "ttcRadar_pkg/ttcRadar_msg.h"
#include "std_msgs/UInt8MultiArray.h"
#include <vector>
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <math.h>
using namespace std;

#define CFG_LOOP_RATE 10 

#define ser_Cfg_Port_Name "/dev/ttyUSB0"
#define ser_Data_Port_Name "/dev/ttyUSB1"

// switch(modeRadar) case ENABLE_RADAR_TTC: ...
#define modeRadar           0
#define ENABLE_RADAR_TTC    1
#define ENABLE_RADAR_MPC    0

#define MMWDEMO_UART_MSG_DETECTED_POINTS			1
#define MMWDEMO_UART_MSG_RANGE_PROFILE				2
#define MMWDEMO_UART_MSG_NOISE_PROFILE				3
#define MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO  7
#define MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS     8
#define MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS_SIDE_INFO   9
#define MMWDEMO_UART_MSG_TRACKERPROC_TARGET_LIST    10
#define MMWDEMO_UART_MSG_TRACKERPROC_TARGET_INDEX   11

// Output of Radar
typedef struct
{
  uint32_t msg_counter = 0;
  bool isObject = false;
  float distance = 10.0f;
} Radar_Output_Struct;

struct structHeader
{
    uint16_t magicWord[8];
    uint32_t version[4];
    uint32_t totalPacketLen = 0;
    uint32_t platform = 0;
    uint32_t frameNumber = 0;
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj = 0;
    uint32_t numTLVs = 0;
    uint32_t subFrameNumber = 0;
    uint32_t numStaticDetectedObj = 0;
    uint32_t idX = 0;
};

struct structTLV
{
    uint32_t type;
    uint32_t length;
    uint32_t idX;
    vector<uint8_t> payload;
};

struct structDetObj
{
	// vector<float> range;
	// vector<float> angle;
	// vector<float> elev;
	vector<float> doppler;

    vector<float> x;
	vector<float> y;
	vector<float> z;

};

struct structStaticDetObj
{
	vector<float> x;
	vector<float> y;
	vector<float> z;
	vector<float> doppler;
};

struct structTargets
{
	vector<uint32_t> tid;
	vector<float> posX;
	vector<float> posY;
	vector<float> velX;
    vector<float> velY;
	vector<float> accX;
	vector<float> accY;
	vector<float> posZ;
    vector<float> velZ;
	vector<float> accZ;
};

union byte2float
{
	int str;
	vector<uint8_t>  myByte;
	vector<float> myFloat;
	~byte2float() {}
};

class ttcRAdarObj
{
    public:
    ttcRAdarObj();
    // explicit
    ttcRAdarObj(ttcRAdarObj &&) {}
    // implicit
    ttcRAdarObj(const ttcRAdarObj&) = default;
    ttcRAdarObj& operator=(const ttcRAdarObj&) = default;
    ~ttcRAdarObj();

    serial::Serial ser_Cfg_Port;
    serial::Serial ser_Data_Port;
    Radar_Output_Struct Output;
    structDetObj ptDetObj;
    structStaticDetObj ptStaticDetObj;
    structTargets ptTargets;
    structTLV tlv;
    vector<float> bufDistance;
    vector<uint16_t> startIdx;

    bool init_cfg_port(void);
    bool init_data_port(void);
    void start_radar(void);
    void stop_radar(void);
    void getDetObj(void);
    void getStaticObj(void);
    void getGtrackTargetList(void);
    bool data_handler(std_msgs::UInt8MultiArray raw_data, uint16_t dataLen);
    float processingPtMinDistance (structHeader frameHeader);


    private:
    void send_cfg(std::string msg);
    void clearPtCloud(void);
    void posframeAvalable(std_msgs::UInt8MultiArray raw_data, vector<uint16_t> &startIdx, uint16_t dataLen);
    structHeader getFrameHeader (uint8_t framePacket[], uint16_t dataLen);
    structTLV getTLV (uint8_t framePacket[], uint32_t numTLVs, uint32_t idX);

};

#endif