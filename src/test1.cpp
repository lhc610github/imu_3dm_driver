#include <iostream>

#include "mscl/Types.h"

#include "mscl/Communication/Connection.h"
#include "mscl/MicroStrain/Inertial/InertialNode.h"
#include "mscl/Exceptions.h"
#include "mscl/MicroStrain/MIP/MipTypes.h"
int main(int argc, char **argv) {
    //create the connection object with port and baud rate
    try{
        mscl::Connection connection = mscl::Connection::Serial("/dev/ttyACM0", 9600);

        //create the InertialNode, passing in the connection
        mscl::InertialNode node(connection);
        //ping the Node
        bool success = node.ping();
        if (success) {
            printf("success!\n");
        }
        // mscl::MipNodeFeatures feature = node.features();//.supportedChannelFields();
        // const mscl::MipTypes::MipChannelFields test_ = node.features().supportedChannelFields(mscl::MipTypes::CLASS_AHRS_IMU);
        // if (success) {
        //     printf("acc success!\n");
        // }
        //put the Inertial Node into its idle state
        node.setToIdle();
        //get all of the active channels for the GPS category on the Node
        mscl::MipChannels activeChs = node.getActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU);

        mscl::MipChannels ahrsImuChs;
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC, mscl::SampleRate::Hertz(100)));
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC, mscl::SampleRate::Hertz(100)));

        // mscl::MipChannels gnssChs;
        // gnssChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_GNSS_LLH_POSITION, mscl::SampleRate::Hertz(1)));

        mscl::MipChannels estFilterChs;
        //estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL, mscl::SampleRate::Hertz(100)));
        estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE, mscl::SampleRate::Hertz(100)));
        estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_COMPENSATED_ACCEL, mscl::SampleRate::Hertz(100)));

        //set the active channels for the different data classes on the Node
        node.setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, ahrsImuChs);
        // node.setActiveChannelFields(mscl::MipTypes::CLASS_GNSS, gnssChs);
        node.setActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER, estFilterChs);
        //start sampling the active channels on the AHRS/IMU class of the Node
        node.enableDataStream(mscl::MipTypes::CLASS_AHRS_IMU, false);
        node.enableDataStream(mscl::MipTypes::CLASS_ESTFILTER);

        //use the resume command to return to the mode before setToIdle
        node.resume();
        //create the InertialNode, passing in the connection
        // mscl::InertialNode node = mscl.InertialNode(connection);
        printf("hello\n");

        while(true)
        {
            //get all the packets that have been collected, with a timeout of 500 milliseconds
            mscl::MipDataPackets packets = node.getDataPackets(500);

            for(mscl::MipDataPacket packet : packets)
            {
                // packet.descriptorSet(); //the descriptor set of the packet
                // packet.timestamp();     //the PC time when this packet was received

                //get all of the points in the packet
                mscl::MipDataPoints points = packet.data();

                for(mscl::MipDataPoint dataPoint : points)
                {
                    dataPoint.channelName();  //the name of the channel for this point
                    std::cout << dataPoint.channelName() << ": ";
                    //dataPoint.storedAs();     //the ValueType that the data is stored as
                    //dataPoint.as_float();     //get the value as a float
                    std::cout << dataPoint.as_float() << std::endl;
                }
            }
        }

    } catch(mscl::Error& e) {
        std::cout << "Error:" << e.what() << std::endl;
    }
}