
/**
 *  目前的目的就是进行一些原理上的验证
 * 
 * 
 * 
 * 
*/
#include "basic.h"

#define ROT 0
#define POS 3
#define VEL 6
#define BIAS_ACC 9
#define BIAS_GYR 12
#define N_ACC 0
#define N_GYR 0


struct ESKFOptions
{
    // IMU Sim GPS
    bool sim_gps;
    int interval;
    double n_ba;
    double n_bw;
    double n_a;
    double n_w;
};

class State;

class EKF
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    #define MEAN_(var) real_state.predict_mean.block<3,1>(var,0)
    
    EKF(const ESKFOptions & _options):options(_options){
        Init();
    }

    void Init();

    void Process();
    
    void AddIMU(const IMUData & imu){imu_pool_.push_back(imu);}
    void AddGPS(const GPSData & gps){gps_pool_.push_back(gps);}
    void SetStart(){ok = true;}
    void SetStop(){ok = false;}

    // 先使用IMU真值进行递推
    bool CutData(std::vector<IMUData> & cutter, GPSData & GPSData);

    void NormalizeIMU(const std::vector<IMUData> & raw_imu_data, std::vector<IMUData> & normal_imu);

    void Predict(const IMUData & normal_imu);
    void Update(const GPSData & gps);
    
    void UpdatePosition(const V3d & pos_obs);
    
    void UpdateVelocity(const V3d & vel_obs);

    void UpdateRotation(const V3d & rot_obs);

    void ResetErrorState();

    State real_state;
    State error_state;

    std::vector<IMUData> imu_pool_;
    std::vector<GPSData> gps_pool_;

    ESKFOptions options;

    bool ok;
    bool init;

    // Noise
    const Eigen::Matrix<double,12,12> Rn;

    const double gravity_scale = 9.79;
    double last_timestamp = -1;
};

void EKF::Init()
{
    real_state = State();
    error_state = State();
    ok = false;
    init = false;
}

void EKF::Process()
{
    while(ok)
    {
        std::vector<IMUData> raw_imu,normal_imu;
        GPSData gps;
        if(CutData(raw_imu,gps))
        {
            for(const IMUData & imu:normal_imu)
            Predict(imu);
            Update(gps);
            ResetErrorState();
        }
    }

}

void EKF::Predict(const IMUData & raw_imu)
{
    //进行速度上的更新
    if(last_timestamp <0)
    {
        last_timestamp = raw_imu.timestamp;
        return ;
    }

    // 进行数据准备
    double delta_timestamp = raw_imu.timestamp - last_timestamp;
    Sophus::SO3d R_b_in_w_so3 = Sophus::SO3d::exp(MEAN_(ROT));
    Eigen::Matrix3d R_b_in_w = R_b_in_w.matrix();

    IMUData imu;
    imu.Gyr = raw_imu.Gyr - MEAN_(BIAS_GYR);
    imu.Acc = raw_imu.Acc - MEAN_(BIAS_ACC);
    double square_timestamp = delta_timestamp * delta_timestamp;
    V3d Acc_in_w = R_b_in_w*imu.Acc - R_b_in_w*(Eigen::Vector3d()<<0,0,gravity_scale).finished();

    // 对位姿进行递推
    MEAN_(POS) += MEAN_(VEL)*delta_timestamp + 0.5 *  Acc_in_w * square_timestamp;

    // 对速度进行递推
    MEAN_(VEL) += Acc_in_w*delta_timestamp;

    // 对角度进行递推
    MEAN_(ROT) = (R_b_in_w_so3*Sophus::SO3d::exp(imu.Gyr*delta_timestamp)).log();

    // 方差准备工作
    Eigen::Matrix<double,15,15> F;
    Eigen::Matrix<double,15,6> G;
    Eigen::Matrix3d real_Gyr_hat = Sophus::SO3d::hat(imu.Gyr*delta_timestamp);
    Eigen::Matrix3d theta_hat = Sophus::SO3d::hat(R_b_in_w_so3.log());

    F.setIdentity();
    G.setIdentity();

    #define F_(var1,var2) F.block<3,3>(var1,var2)
    #define G_(var1,var2) G.block<3,3>(var1,var2)

    // 速度方差递推
    F_(VEL,VEL) = Eigen::Matrix3d::Identity();
    F_(VEL,ROT) = - Sophus::SO3d::hat(Acc_in_w * delta_timestamp);
    F_(VEL,BIAS_ACC) = -R_b_in_w * delta_timestamp;
    G_(VEL,N_ACC) = -F_(VEL,BIAS_ACC);

    // 位置方差递推
    F_(POS,POS) = Eigen::Matrix3d::Identity();
    F_(POS,VEL) = Eigen::Matrix3d::Identity() * delta_timestamp;
    F_(POS,BIAS_ACC) = -0.5 * R_b_in_w * square_timestamp;
    F_(POS,ROT) = -0.5*Sophus::SO3d::hat(Acc_in_w * square_timestamp);
    G_(POS,N_ACC) = -F_(POS,BIAS_ACC);

    // 角度方差递推
    // 这里直接参考doc 内文章的实现，取一阶近似
    F_(ROT,ROT) = -0.5*real_Gyr_hat;
    F_(ROT,BIAS_GYR) = - Eigen::Matrix3d::Identity()*delta_timestamp
                       - 0.5*theta_hat*delta_timestamp;
    G_(ROT,BIAS_GYR) = - F_(ROT,BIAS_GYR);

    real_state.predict_cov = F*real_state.predict_cov*F.transpose() + G*Rn*G.transpose();
}

void EKF::Update(const GPSData & gps)
{
    
    // 这里主要使用和EKF对等的优化方法进行更新，具体内容见 doc.md

    Eigen::Matrix<double,6,15> K;

    Eigen::Matrix<double,6,3> Hessian;;
    Eigen::Matrix<double,6,15> Gradient;

    // res_obs position 位置观测上的residual
    Eigen::Matrix<double,3,1> pos_res;
    Eigen::Matrix<double,3,15> J_position;
    
    pos_res = gps.position - MEAN_(POS);
    J_position = 

    // TODO 为了之后进行 IKF的拓展 这里最好使用 little_g2o 直接使用优化模型进行更新

}