/**
 *    进行一层包装
 * 
 * 
*/


#include "imu.h"
#include "basic.h"

// 一个 DataUnit For a period of propagation and a Correct
class DataUnit
{
    std::vector<IMUData> imu_pool;
    GPSData gps;
};

class Simulator
{
    public:

    std::vector<DataUnit> GenerateTestData();   
};

std::vector<DataUnit> Simulator::GenerateTestData()
{
    Param params;
    IMU imuGen(params);

    // create imu data
    // imu pose gyro acc
    std::vector< MotionData > imudata;
    std::vector< MotionData > imudata_noise;
    for (float t = params.t_start; t<params.t_end;) {
        MotionData data = imuGen.MotionModel(t);
        imudata.push_back(data);

        // add imu noise
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        imudata_noise.push_back(data_noise);

        t += 1.0/params.imu_frequency;
    }
    imuGen.init_velocity_ = imudata[0].imu_velocity;
    imuGen.init_twb_ = imudata.at(0).twb;
    imuGen.init_Rwb_ = imudata.at(0).Rwb;

    
}
