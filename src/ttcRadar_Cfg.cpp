#include "ttcRadar_Cfg.h"

ttcRAdarObj::ttcRAdarObj()
{

}

ttcRAdarObj::~ttcRAdarObj()
{

}


bool ttcRAdarObj::init_cfg_port(void)
{
    // Init Radar Config Port
    try
    {
        ser_Cfg_Port.setPort(ser_Cfg_Port_Name);
        ser_Cfg_Port.setBaudrate(115200);
        serial::Timeout to1 = serial::Timeout::simpleTimeout(1000);
        ser_Cfg_Port.setTimeout(to1);
        ser_Cfg_Port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open config port ");
        return false;
    }
    if(ser_Cfg_Port.isOpen()){
        ROS_INFO_STREAM("Radar Config Port initialized");
    }else{
        return false;
    }
}

bool ttcRAdarObj::init_data_port(void)
{
    // Init Radar Data Port
    try
    {
        ser_Data_Port.setPort(ser_Data_Port_Name);
        ser_Data_Port.setBaudrate(921600);
        serial::Timeout to2 = serial::Timeout::simpleTimeout(1000);
        ser_Data_Port.setTimeout(to2);
        ser_Data_Port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open data port ");
        return false;
    }
    if(ser_Data_Port.isOpen()){
        ROS_INFO_STREAM("Radar Data Port initialized");
    }else{
        return 1;
    }
}

void ttcRAdarObj::send_cfg(std::string msg)
{
    ros::Rate loop_rate1(CFG_LOOP_RATE);

    ser_Cfg_Port.write(msg + "\n");
    ROS_INFO_STREAM("Send: " << msg);
    loop_rate1.sleep();

    if(ser_Cfg_Port.available())
    {
      std_msgs::String result;
      result.data = ser_Cfg_Port.read(ser_Cfg_Port.available());
      ROS_INFO_STREAM("-Read: " << result.data);
      //read_pub.publish(result);
    }
    loop_rate1.sleep();
}

void ttcRAdarObj::start_radar(void)
{
    std::string msg;

    msg = "sensorStop";
    send_cfg(msg);

    msg = "flushCfg";
    send_cfg(msg);

    msg = "dfeDataOutputMode 1";
    send_cfg(msg);

    msg = "channelCfg 15 7 0";
    send_cfg(msg);

    msg = "adcCfg 2 1";
    send_cfg(msg);

    msg = "adcbufCfg -1 0 1 1 1";
    send_cfg(msg);

    msg = "profileCfg 0 60 100 25 69 0 0 50 1 256 6000 0 0 30";
    send_cfg(msg);

    msg = "chirpCfg 0 0 0 0 0 0 0 1";
    send_cfg(msg);

    msg = "chirpCfg 1 1 0 0 0 0 0 4";
    send_cfg(msg);

    msg = "chirpCfg 2 2 0 0 0 0 0 2";
    send_cfg(msg);

    // 55 is 55ms delay between 2 output frame
    msg = "frameCfg 0 1 32 0 55 1 0";
    send_cfg(msg);

    msg = "lowPower 0 0";
    send_cfg(msg);

    msg = "guiMonitor -1 1 0 0 0 0 0";
    send_cfg(msg);

    //Threshold scale [0..100]
    msg = "cfarCfg -1 0 2 8 4 3 0 20 1";
    send_cfg(msg);
    msg = "cfarCfg -1 1 0 8 4 4 1 15 1";
    send_cfg(msg);

    msg = "multiObjBeamForming -1 1 0.5";
    send_cfg(msg);

    msg = "clutterRemoval -1 1";
    send_cfg(msg);

    msg = "calibDcRangeSig -1 0 -5 8 256";
    send_cfg(msg);

    msg = "extendedMaxVelocity -1 0";
    send_cfg(msg);

    msg = "bpmCfg -1 0 0 1";
    send_cfg(msg);

    msg = "lvdsStreamCfg  -1 0 0 0";
    send_cfg(msg);

    msg = "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0";
    send_cfg(msg);

    msg = "measureRangeBiasAndRxChanPhase 0 1.5 0.2";
    send_cfg(msg);

    msg = "CQRxSatMonitor 0 3 15 125 0";
    send_cfg(msg);

    msg = "CQSigImgMonitor 0 115 6";
    send_cfg(msg);

    msg = "analogMonitor 0 0";
    send_cfg(msg);

    // View config (degrees) : [ -1 <minAzimuthDeg> <maxAzimuthDeg> <minElevationDeg> <maxElevationDeg> ]
    msg = "aoaFovCfg -1 -40 40 -10 10";
    send_cfg(msg);

    // Config point filtering in range direction (meter)
    msg = "cfarFovCfg -1 0 0 20.0";
    send_cfg(msg);

    // Config point filtering in Doppler direction (meter/sec)
    msg = "cfarFovCfg -1 1 -10 10";
    send_cfg(msg);

    // msg = "calibData 0 0 0";
    // send_cfg(msg);

    // *****************TRACKING COMMANDS*****************************
    // https://dev.ti.com/tirex/explore/content/mmwave_industrial_toolbox_4_7_0/labs/people_counting/docs/3D_people_counting_tracker_layer_tuning_guide.pdf
    
    msg = "staticBoundaryBox -8 8 0 20 -1 1";
    send_cfg(msg);

    msg = "boundaryBox -8 8 0 20 -1 1";
    send_cfg(msg);

    msg = "gatingParam 3 2 2 2 20";
    send_cfg(msg);

    msg = "stateParam 15 10 10 20 5";
    send_cfg(msg);

    msg = "allocationParam 200 150 0.05 10 0.3 10";
    send_cfg(msg);

    msg = "maxAcceleration 2 2 2";
    send_cfg(msg);

    msg = "trackingCfg 1 2 250 20 100 50 55 90";
    send_cfg(msg);

    // *****************STATIC DETECTION COMMANDS*********************
    msg = "heatmapGenCfg -1 0  0 40 130 60.0 3.0 10";
    send_cfg(msg);

    msg = "staticDetectionCfg -1 0 -50.0 +50.0 -20.0 20.0 0.7 6.0 0.2 4 20.0";
    send_cfg(msg);

    msg = "sensorStart";
    send_cfg(msg);
}

void ttcRAdarObj::stop_radar(void)
{
    std::string msg;
    msg = "sensorStop";
    send_cfg(msg);
}

structHeader ttcRAdarObj::getFrameHeader (uint8_t framePacket[], uint16_t dataLen)
{
	structHeader frameHeader;

	// check that all packet has been read
    frameHeader.totalPacketLen = framePacket[12] + framePacket[13] * 256.0 + framePacket[14] * 65536.0 + framePacket[15] * 1.6777216E+7;
	uint32_t idX = 0;

	// read the header
	if (frameHeader.totalPacketLen == dataLen) 
	{
		// word array to convert 4 bytes to a 32 bit number
        // word = [1, 2**8, 2**16, 2**24]

        // Initialize the pointer index
        for (auto idX = 0; idX < 8; idX++)
		{
			frameHeader.magicWord[idX] = framePacket[idX];
		}
		idX += 8;
		for (auto idX = 0; idX < 4; idX++)
		{
			frameHeader.version[idX] = framePacket[idX + 8];
		}
		idX += 4;
        frameHeader.totalPacketLen = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.platform = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.frameNumber = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.timeCpuCycles = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.numDetectedObj = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.numTLVs = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
		frameHeader.subFrameNumber = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.numStaticDetectedObj  = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
 	}
	frameHeader.idX = idX;

    ROS_INFO("totalPacketLen: %u", frameHeader.totalPacketLen);
    ROS_INFO("platform: %u", frameHeader.platform);
    ROS_INFO("frameNumber: %u", frameHeader.frameNumber);
    ROS_INFO("numDetectedObj: %u", frameHeader.numDetectedObj);
    ROS_INFO("numTLVs: %u", frameHeader.numTLVs);
    ROS_INFO("idX: %u", frameHeader.idX);
    // for(auto i=0; i<300; i++)
    // {
    //     ROS_INFO("frame %d: %u", i, framePacket[i]);
    // }

	return frameHeader;
}

void ttcRAdarObj::clearPtCloud(void)
{

    tlv.payload.clear();
    ptStaticDetObj.x.clear();
    ptStaticDetObj.y.clear();
    ptStaticDetObj.z.clear();
    ptStaticDetObj.doppler.clear();

    ptDetObj.x.clear();
    ptDetObj.y.clear();
    ptDetObj.z.clear();
    ptDetObj.doppler.clear();

    ptTargets.tid.clear();
    // ptTargets.posX.push_back(data.myFloat[i * 4]);
    ptTargets.posX.clear();
    ptTargets.posY.clear();
    ptTargets.velX.clear();
    ptTargets.velY.clear();
    ptTargets.accX.clear();
    ptTargets.accY.clear();
    ptTargets.posZ.clear();
    ptTargets.velZ.clear();
    ptTargets.accZ.clear();

}

void ttcRAdarObj::getDetObj(void)
{
    int numDetectedObj = tlv.length/16;
    byte2float data = {0};

    if (numDetectedObj)
    {
        // Convert 4byte to float
        for (auto i = 0; i < tlv.length; i++)
        {
            data.myByte.push_back(tlv.payload[i]);
        }

        for (auto i = 0; i < numDetectedObj; i++)
        {
            // ptDetObj.range.push_back(data.myFloat[i * 4]);
            // ptDetObj.angle.push_back(data.myFloat[i * 4 + 1]);
            // ptDetObj.elev.push_back(data.myFloat[i * 4 + 2]);
            ptDetObj.doppler.push_back(data.myFloat[i * 4 + 3]);
            
            ptDetObj.z.push_back(   data.myFloat[i * 4]             *   sin(data.myFloat[i * 4 + 2])                        );
            ptDetObj.y.push_back(   cos(data.myFloat[i * 4 + 2])    *   cos(data.myFloat[i * 4 + 2]) * data.myFloat[i * 4]  );
            ptDetObj.x.push_back(   sin(data.myFloat[i * 4 + 2])    *   cos(data.myFloat[i * 4 + 2]) * data.myFloat[i * 4]  );

        }
    }
}

void ttcRAdarObj::getStaticObj(void)
{
    int numDetectedObj = tlv.length/16;
    byte2float data = {0};

    if (numDetectedObj)
    {
        // Convert 4byte to float
        for (auto i = 0; i < tlv.length; i++)
        {
            data.myByte.push_back(tlv.payload[i]);
        }

        for (auto i = 0; i < numDetectedObj; i++)
        {
            ptStaticDetObj.x.push_back(data.myFloat[i * 4]);
            ptStaticDetObj.y.push_back(data.myFloat[i * 4 + 1]);
            ptStaticDetObj.z.push_back(data.myFloat[i * 4 + 2]);
            ptStaticDetObj.doppler.push_back(data.myFloat[i * 4 + 3]);
        }
    }
}

void ttcRAdarObj::getGtrackTargetList(void)
{
    int numDetectedObj = tlv.length/16;
    byte2float data = {0};

    if (numDetectedObj)
    {
        // Convert 4byte to float
        for (auto i = 0; i < tlv.length; i++)
        {
            data.myByte.push_back(tlv.payload[i]);
        }

        for (auto i = 0; i < numDetectedObj; i++)
        {
            ptTargets.tid.push_back(tlv.payload[0]*1 + tlv.payload[1]*256.0 + tlv.payload[2]*65536.0 + tlv.payload[3]*1.6777216E+7);
            // ptTargets.tid.push_back(data.myFloat[i * 10 + 0]);
            ptTargets.posX.push_back(data.myFloat[i * 10 + 1]);
            ptTargets.posY.push_back(data.myFloat[i * 10 + 2]);
            ptTargets.velX.push_back(data.myFloat[i * 10 + 3]);
            ptTargets.velY.push_back(data.myFloat[i * 10 + 4]);
            ptTargets.accX.push_back(data.myFloat[i * 10 + 5]);
            ptTargets.accY.push_back(data.myFloat[i * 10 + 6]);
            ptTargets.posZ.push_back(data.myFloat[i * 10 + 7]);
            ptTargets.velZ.push_back(data.myFloat[i * 10 + 8]);
            ptTargets.accZ.push_back(data.myFloat[i * 10 + 9]);
        }
    }
}

structTLV ttcRAdarObj::getTLV (uint8_t framePacket[], uint32_t numTLVs, uint32_t idX)
{

    // clear all the elements of the vector container
    clearPtCloud();

	// read all (numTLVs)TLVs to ptCloud
	for (auto tlvIdx = 0; tlvIdx < numTLVs; tlvIdx++)
	{
        // check the header of the TLV message
		tlv.type = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
		idX += 4;
		tlv.length = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
		idX += 4;
		for (auto i = 0; i < tlv.length ; i++)
			{
				tlv.payload.push_back(framePacket[idX + i]);
			}
		idX += tlv.length;
        tlv.idX = idX;
                ROS_INFO("type: %u  ---------------------", tlv.type);
                ROS_INFO("len: %u  ---------------------", tlv.length);

		switch (tlv.type)
		{
            // getGtrackPtCloud() == 1
			case MMWDEMO_UART_MSG_DETECTED_POINTS :
			{
                getDetObj();
			}
			break;

            // getStaticObj == 8
            case MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS :
			{
                // getStaticObj();
			}
			break;

            // getGtrackTargetList == 10
            case MMWDEMO_UART_MSG_TRACKERPROC_TARGET_LIST :
			{
                getGtrackTargetList();
			}
			break;

			case MMWDEMO_UART_MSG_RANGE_PROFILE:
			break;
			case MMWDEMO_UART_MSG_NOISE_PROFILE:
			break;
			case MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO:
			break;
			default:
			break;
		}
	}

    return tlv;
}

float ttcRAdarObj::processingPtMinDistance (structHeader frameHeader)
{

    // sorting vector distance in increasing order
    sort(ptDetObj.y.begin(), ptDetObj.y.end());

    // para config output
    float delta = 0.3;
    float numRangePt = 1;
    float ptMinDistance = 0;
    float rangePt = (float)(ptDetObj.y.size())/3;
    ROS_INFO("numDetected = %zu", ptDetObj.y.size());

    for (int i = 0; i < ptDetObj.y.size(); i++)
    {
        ROS_INFO("y= %f", ptDetObj.y[i]);
    }

    // check the numDetected, 
    if (ptDetObj.y.size() > 1)
    {
        for (auto i = 0; i < ptDetObj.y.size() - 1; i++)
        {
            ptMinDistance = ptDetObj.y[i];
            numRangePt = 1;
            for (auto j = 0; j < ptDetObj.y.size() - 1 - i; j++)
            {
                if(abs(ptDetObj.y[i + j +1] - ptDetObj.y[i]) < delta)
                {
                    ptMinDistance += ptDetObj.y[i + j +1];
                    numRangePt++;
                }
            }
            ROS_INFO("numRangePt = %f", numRangePt);

            if(numRangePt >= rangePt)
            {
                ptMinDistance = ptMinDistance/(int)numRangePt;
                break;
            }
            else
            {
                ptMinDistance = ptDetObj.y[0];
            }
        }
    }
    else if(ptDetObj.y.size() == 1)
    {
        ptMinDistance = ptDetObj.y[0];
    }
    else
    {
        ptMinDistance = 10.0;
    }
    ROS_INFO("outDisTmp = %f", ptMinDistance);
    
    // fillter 2
    float delta_2 = 0.8;
    float numRangePt_2 = 1;
    bufDistance.push_back(ptMinDistance); // vector global

    // buffer output distance has 5 elements
    if (bufDistance.size() == 6)
    {
        for (int i = 0; i < bufDistance.size() - 1; i++)
        {
        ROS_INFO("bufDis= %f", bufDistance[i+1]);
        }

        bufDistance.erase(bufDistance.begin());
        float rangePt_2 = (float)(bufDistance.size())/2; // how many elements to fit 

        for (auto i = 0; i < bufDistance.size() - 1; i++)
        {
            ptMinDistance = bufDistance[i];
            numRangePt_2 = 1;
            for (auto j = 0; j < bufDistance.size() - 1 - i; j++)
            {
                if(abs(bufDistance[i + j +1] - bufDistance[i]) < delta_2)
                {
                    ptMinDistance += bufDistance[i + j +1];
                    numRangePt_2++;
                }
            }
            ROS_INFO("numRangePt_2 = %f", numRangePt_2);

            if(numRangePt_2 >= rangePt_2)
            {
                ptMinDistance = ptMinDistance/(int)numRangePt_2;
                break;
            }
            else
            {
                ptMinDistance = bufDistance[0];
                ROS_INFO("no filter 2");
            }
        }
    }

    return ptMinDistance;
}

void ttcRAdarObj::posframeAvalable(std_msgs::UInt8MultiArray raw_data, vector<uint16_t> &startIdx, uint16_t dataLen)
{
    startIdx.clear();

    // magic word = [2,1,4,3,6,5,8,7]
    for (uint32_t i = 0; i < dataLen - 7; i++)
    {
        if (raw_data.data[i] == 2 && raw_data.data[i+1] == 1 && raw_data.data[i+2] == 4 && raw_data.data[i+3] == 3 
            && raw_data.data[i+4] == 6 && raw_data.data[i+5] == 5 && raw_data.data[i+6] == 8 && raw_data.data[i+7] == 7)
        {
            startIdx.push_back(i);
            ROS_INFO("i = %u",i);
        }
    }
}

bool ttcRAdarObj::data_handler( std_msgs::UInt8MultiArray raw_data, uint16_t dataLen)
{
    bool is_data_ok = false;

    // Check for all possible locations of the magic word to startIdx
    posframeAvalable(raw_data, startIdx, dataLen);
    u_int16_t numframesAvailable = startIdx.size();

    // Processing
    if (numframesAvailable > 0)
    {
        ROS_INFO("have %u numframesAvailable", numframesAvailable);
        is_data_ok = true;

        // Check that startIdx is not empty // framePacket has executed only 1 frame
        startIdx.push_back(dataLen);
                        ROS_INFO("startIdx 0 =  %u", startIdx[0]);
                         ROS_INFO("startIdx 1 =  %u", startIdx[1]);


        uint8_t framePacket[(startIdx[1] - startIdx[0])];
       
        //Remove the data before the first start index
        for (auto i = 0; i < (startIdx[1] - startIdx[0]); i++)
        {
            framePacket[i] = raw_data.data[startIdx[0] + i];
        }
        //update dataLen
        dataLen = startIdx[1] - startIdx[0];
                ROS_INFO("DataLen =  %u", dataLen);

        // Read the Header messages
        structHeader frameHeader = getFrameHeader(framePacket, dataLen);
        uint32_t idX = frameHeader.idX;

        // Check is_data_ok
        if (frameHeader.numTLVs != 0)
        {
            // Read the TLV messages
            structTLV tlv = getTLV(framePacket, frameHeader.numTLVs, idX);
            idX = tlv.idX;

            // processing output
            switch (modeRadar)
            {
                case ENABLE_RADAR_TTC:
                {
                    

                }
                break;

                case ENABLE_RADAR_MPC:
                {

                    float ptMinDistance = processingPtMinDistance(frameHeader);
                    // update output
                    if (frameHeader.numDetectedObj)
                    {
                        Output.isObject = true;
                        Output.msg_counter++;
                        Output.distance = ptMinDistance;
                        ROS_INFO("distance ============= %f",ptMinDistance);
                    }
                    else
                    {
                        Output.isObject = false;
                        Output.distance = ptMinDistance;
                        ROS_INFO("distance ============= %f", ptMinDistance);
                    }

                }
                break;

                default:
                break;
            }
        }
        else
        {
            // is_data_ok = false;
            Output.isObject = false;
            ROS_INFO("distance ============= %f", Output.distance);
        }
        
    }

    return is_data_ok;
}
