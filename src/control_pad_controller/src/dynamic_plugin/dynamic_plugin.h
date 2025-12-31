#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/regressor.hpp>
#include <future>
#include "robot_handle.h"
#include "singleton.hpp"

class DynamicPlugin : public Singleton<DynamicPlugin>{
    friend class Singleton<DynamicPlugin>;
    
public:
    struct DependencyAnalysisResult {
        Eigen::MatrixXd Pb; // independant column selection matrix
        Eigen::MatrixXd Pd; // dependant column selection matrix
        Eigen::MatrixXd Kd; // relation matrix
    };
    
    DynamicPlugin();

public:
    void identify(const size_t& db_start_index, const size_t& db_end_index);
    std::optional<JointsTorque> rnea(const JointsPosition& q, const JointsVelocity& v, const JointsAcceleration& a);
    std::optional<JointsTorque> currentPoseStableTorque(const JointsPosition& q);
    std::pair<std::vector<double>, std::vector<double>> firstOrderMomentum(const std::vector<double>& q, const std::vector<double>& v, const std::vector<double>& t);
    std::pair<Eigen::VectorXd, Eigen::VectorXd> firstOrderMomentum(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& t);
    const bool& isReady() const;

    const Eigen::MatrixXd& dependenceComputation();
    const Eigen::MatrixXd& getBaseParams();
    void setParams(const Eigen::MatrixXd &base, const Eigen::MatrixXd &friction, const Eigen::MatrixXd &Pb, const Eigen::MatrixXd &Pd, const Eigen::MatrixXd &Kd);
    const Eigen::MatrixXd& getFrictionParams();
    const Eigen::MatrixXd& getDepPb();
    const Eigen::MatrixXd& getDepPd();
    const Eigen::MatrixXd& getDepKd();

private:
    Eigen::VectorXd calculateExternalCartesianForce(const Eigen::VectorXd &q, const Eigen::VectorXd& joint_torques);
    DependencyAnalysisResult calculateDynamicParamsDependence(int sample_num, double thre);
    void shuffleVector(Eigen::VectorXd& vec, const Eigen::VectorXd& upper_limit, Eigen::VectorXd lower_limit = Eigen::VectorXd{});
    // friction helpers
    Eigen::VectorXd computeFrictionTorque(const Eigen::VectorXd &v);
    void fillFrictionRegressor(Eigen::MatrixXd &block, const Eigen::VectorXd &v);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    DependencyAnalysisResult dep_res_;
    Eigen::MatrixXd all_params_;
    Eigen::MatrixXd base_params_model_;
    Eigen::MatrixXd base_params_;
    Eigen::MatrixXd friction_params_;
    size_t nq_;
    Eigen::MatrixXd K0_;// first order momentum observer gains
    std::future<DependencyAnalysisResult> dep_future_;
    bool is_ready_{false};
};
