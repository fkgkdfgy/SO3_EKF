
/**
 *  目前的目的就是进行一些原理上的验证
 * 
 * 
 * 
 * 
*/
#include "basic.h"

using namespace std;


#define ROT 0
#define POS 3
#define VEL 6
#define BIAS_ACC 9
#define BIAS_GYR 12
#define N_ACC 0
#define N_GYR 3
#define N_BA 6
#define N_BG 9

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

struct EKFLog
{
};

struct State
{
    State(){
        predict_mean.setZero();
        predict_cov.setZero();
    }

    void UpdateCov(const Eigen::Matrix<double,15,15> & new_cov)
    {
        predict_cov = new_cov;
    }
    void UpdateMean(const Eigen::Matrix<double,15,1> & delta_x)
    {
        Eigen::Vector3d old_rot = predict_mean.block<3,1>(ROT,0);
        predict_mean += delta_x;
        predict_mean.block<3,1>(ROT,0) = (Sophus::SO3d::exp(old_rot)*Sophus::SO3d::exp(delta_x.block<3,1>(ROT,0))).log();
    }

    Eigen::Matrix<double,15,1> predict_mean;
    Eigen::Matrix<double,15,15> predict_cov;
};

class EKF
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    #define MEAN_(var) real_state.predict_mean.block<3,1>(var,0)
    
    EKF() = delete;

    explicit EKF(const ESKFOptions & _options):options(_options){
        Init();
    }

    void Init();

    void Process();

    void RunOnce(const DataUnit & input);

    void SetPosition(const V3d & init_pos){MEAN_(POS) = init_pos;}
    void SetRotation(const V3d & init_rot){MEAN_(ROT) = init_rot;}
    void SetVelocity(const V3d & init_vel){MEAN_(VEL)= init_vel;}

    
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
    Eigen::Matrix<double,12,12> Rn;

    const double gravity_scale = 9.81;
    double last_timestamp = -1;
};

void EKF::Init()
{
    real_state = State();
    error_state = State();
    ok = false;
    init = false;

    Rn.block<3,3>(N_ACC,N_ACC) = Eigen::Matrix3d::Identity() * options.n_a;
    Rn.block<3,3>(N_GYR,N_GYR) = Eigen::Matrix3d::Identity() * options.n_w;
    Rn.block<3,3>(N_BA,N_BA) = Eigen::Matrix3d::Identity() * options.n_ba;
    Rn.block<3,3>(N_BG,N_BG) = Eigen::Matrix3d::Identity() * options.n_bw;

}


void EKF::RunOnce(const DataUnit & input)
{
    for(const IMUData & imu:input.imu_pool)
    Predict(imu);
    Update(input.gps);
}


void EKF::Predict(const IMUData & raw_imu)
{
    //进行速度上的更新

    // 进行数据准备
    double delta_timestamp = 0.005;
    Sophus::SO3d R_b_in_w_so3 = Sophus::SO3d::exp(MEAN_(ROT));
    Eigen::Matrix3d R_b_in_w = R_b_in_w_so3.matrix();

    IMUData imu;
    imu.Gyr = raw_imu.Gyr - MEAN_(BIAS_GYR);
    imu.Acc = raw_imu.Acc - MEAN_(BIAS_ACC);
    double square_timestamp = delta_timestamp * delta_timestamp;

    V3d Acc_in_w = R_b_in_w*imu.Acc - (Eigen::Vector3d()<<0,0,gravity_scale).finished();;

    // 对位姿进行递推
    MEAN_(POS) += MEAN_(VEL)*delta_timestamp + 0.5 *  Acc_in_w * square_timestamp;

    // 对速度进行递推
    MEAN_(VEL) += Acc_in_w*delta_timestamp;

    // 对角度进行递推
    MEAN_(ROT) = (R_b_in_w_so3*Sophus::SO3d::exp(imu.Gyr*delta_timestamp)).log();

    // 方差准备工作
    Eigen::Matrix<double,15,15> F;
    Eigen::Matrix<double,15,12> G;
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
    F_(POS,ROT) = -0.5*Sophus::SO3d::hat(Acc_in_w * square_timestamp);
    F_(POS,BIAS_ACC) = -0.5 * R_b_in_w * square_timestamp;
    G_(POS,N_ACC) = -F_(POS,BIAS_ACC);

    // 角度方差递推
    // 这里直接参考doc 内文章的实现，取一阶近似
    F_(ROT,ROT) = -0.5*real_Gyr_hat + Eigen::Matrix3d::Identity();
    F_(ROT,BIAS_GYR) = - Eigen::Matrix3d::Identity()*delta_timestamp
                       - 0.5*theta_hat*delta_timestamp;
    G_(ROT,N_GYR) = - F_(ROT,BIAS_GYR);
 
    // 没有进行Bias上面的添加 导致方差矩阵不可逆
    F_(BIAS_ACC,BIAS_ACC) = Eigen::Matrix3d::Identity();
    F_(BIAS_GYR,BIAS_GYR) = Eigen::Matrix3d::Identity();

    G_(BIAS_ACC,N_BA) = Eigen::Matrix3d::Identity()*delta_timestamp;
    G_(BIAS_GYR,N_BG) = Eigen::Matrix3d::Identity()*delta_timestamp;

    // cout<<"--------- Old Position Covariance: ------------"<<endl<<real_state.predict_cov.block<3,3>(POS,POS).diagonal().transpose()<<endl;
    // cout<<"--------- New Add Covariance Item: ----------"<<endl<<(G*Rn*G.transpose()).diagonal().transpose()<<endl;
    // cout<<"--------- NEW Basic Covariance Item: -----------"<<endl<<(F*real_state.predict_cov*F.transpose()).block<3,3>(POS,POS).diagonal().transpose()<<endl;
    real_state.predict_cov = F*real_state.predict_cov*F.transpose() + G*Rn*G.transpose();
//     cout<<"--------- New Position Covariance: ------------"<<endl<<real_state.predict_cov.block<3,3>(POS,POS).diagonal().transpose()<<endl;
}

void EKF::Update(const GPSData & gps)
{
    std::cout<<"------- EKF output before correct:-------"<<std::endl;
    std::cout<<MEAN_(POS).transpose()<<"\t"<< MEAN_(ROT).transpose()<<"\t"<<MEAN_(BIAS_ACC).transpose()<<"\t" << MEAN_(BIAS_GYR).transpose()<<std::endl;
    // std::cout<<"------- real output:-------"<<std::endl;
    // std::cout<<gps.position.transpose()<<"\t"<<gps.orientation.transpose()<<std::endl; 

    // 这里主要使用和EKF对等的优化方法进行更新，具体内容见 doc.md

    Eigen::Matrix<double,6,15> K;

    Eigen::Matrix<double,6,3> Hessian;;
    Eigen::Matrix<double,6,15> Gradient;

    // res_obs position 位置观测上的residual
    Eigen::Matrix<double,3,1> pos_res;
    Eigen::Matrix<double,3,15> J_position;

    J_position.setZero();
    pos_res = gps.position - MEAN_(POS);
    J_position.block<3,3>(0,3) = -Eigen::Matrix3d::Identity();
    
    // res_obs orientation 位姿上的观测
    Eigen::Matrix<double,3,1> rot_res;
    Eigen::Matrix<double,3,15> J_orientation;
    Sophus::SO3d rot_res_so3 = Sophus::SO3d::exp(gps.orientation).inverse() * Sophus::SO3d::exp(MEAN_(ROT));

    rot_res = (rot_res_so3).log();
    J_orientation.block<3,3>(0,ROT) = rot_res_so3.Adj();

    // 添加先验
    Eigen::Matrix<double,15,1> piror_res;
    Eigen::Matrix<double,15,15> J_piror;
    // 优化第一次迭代默认 piror_res == 0 J_piror 默认单位阵
    piror_res.setZero();
    J_piror.setIdentity();


    // 混合求解
    Eigen::Matrix<double,15,1> delta_x,hybrid_gradient;
    Eigen::Matrix<double,15,15> hybrid_hessian;

    hybrid_hessian.setZero();

    hybrid_hessian += J_position.transpose() * gps.postion_info * J_position;
    hybrid_hessian += J_orientation.transpose() * gps.orientation_info * J_orientation;
    hybrid_hessian += real_state.predict_cov.inverse();

    cout<<"-----Covariance:------"<<endl<<real_state.predict_cov.inverse().diagonal().transpose()<<endl;
    
    hybrid_gradient.setZero();
    hybrid_gradient += -J_position.transpose() * gps.postion_info * pos_res;
    hybrid_gradient += -J_orientation.transpose() * gps.orientation_info * rot_res;

    delta_x = hybrid_hessian.ldlt().solve(hybrid_gradient);

    // hybird_hessian 条件数查看

    cout<<"------Hessian:------"<<endl<<hybrid_hessian.diagonal().transpose()<<endl;

    real_state.UpdateMean(delta_x);
    real_state.UpdateCov(hybrid_hessian.inverse());
    std::cout<<"------- EKF output:-------"<<std::endl;
    std::cout<<MEAN_(POS).transpose()<<"\t"<< MEAN_(ROT).transpose()<<"\t"<<MEAN_(BIAS_ACC).transpose()<<"\t" << MEAN_(BIAS_GYR).transpose()<<std::endl;
    std::cout<<"------- real output:-------"<<std::endl;
    std::cout<<gps.position.transpose()<<"\t"<<gps.orientation.transpose()<<std::endl; 
    std::cout<<"------- NEW Covariance: --------:"<<endl<< hybrid_hessian.inverse().diagonal().transpose()<<endl;
}