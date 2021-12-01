/**
 *    进行一层包装
 * 
 * 
*/


#include "imu.h"
#include "basic.h"

class Simulator
{
    public:

    std::vector<DataUnit> GenerateTestData();   
    Param params;
    Eigen::Vector3d init_pos;
    Eigen::Vector3d init_rot;
    Eigen::Vector3d init_vec;
};

std::vector<DataUnit> Simulator::GenerateTestData()
{
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

    std::vector<DataUnit> result;
    int count = 0;
    DataUnit tmp_unit;

    init_pos = imudata.front().twb;
    init_rot = Sophus::SO3d(imudata.front().Rwb).log();
    init_vec = imudata.front().imu_velocity;

    for(int i =0;i<imudata_noise.size();i++)
    {
        const auto & e = imudata_noise[i];
        IMUData tmp_imu;
        tmp_imu.Acc = e.imu_acc;
        tmp_imu.Gyr = e.imu_gyro;
        tmp_imu.timestamp = e.timestamp;
        tmp_unit.imu_pool.push_back(tmp_imu);
        count++;
        if(count == params.imu_gps_interval)
        {
            GPSData tmp_gps;
            tmp_gps.timestamp = e.timestamp;
            tmp_gps.position = e.twb;
            tmp_gps.orientation = Sophus::SO3d(e.Rwb).log();
            tmp_gps.position_cov = Eigen::Matrix3d::Identity()* 1e-9 * 1e-9;
            tmp_gps.orientation_cov = Eigen::Matrix3d::Identity()*1e-9 * 1e-9;
            tmp_gps.postion_info = tmp_gps.position_cov.inverse();
            tmp_gps.orientation_info = tmp_gps.orientation_cov.inverse();
            tmp_unit.gps = tmp_gps;
            result.push_back(tmp_unit);
            tmp_unit.imu_pool.clear();
            count =0;
        }
    }
    return result;
}
