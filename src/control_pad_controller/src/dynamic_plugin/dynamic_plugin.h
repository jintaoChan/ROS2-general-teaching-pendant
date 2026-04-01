#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/regressor.hpp>
#include <future>
#include "i_dynamics_service.h"
#include "robot_ports.h"

class DynamicPlugin : public IDynamicsService {
public:
    struct DependencyAnalysisResult {
        Eigen::MatrixXd Pb; // independant column selection matrix
        Eigen::MatrixXd Pd; // dependant column selection matrix
        Eigen::MatrixXd Kd; // relation matrix
    };
    
    explicit DynamicPlugin(IRobotStateProvider* state_port);

public:
    void identify(const size_t& db_start_index, const size_t& db_end_index) override;
    std::optional<JointsTorque> rnea(const JointsPosition& q, const JointsVelocity& v, const JointsAcceleration& a) override;
    std::optional<JointsTorque> currentPoseStableTorque(const JointsPosition& q) override;
    const bool& isReady() const override;

    const Eigen::MatrixXd& dependenceComputation() override;
    const Eigen::MatrixXd& getBaseParams() override;
    void setParams(const Eigen::MatrixXd &base, const Eigen::MatrixXd &friction, const Eigen::MatrixXd &Pb, const Eigen::MatrixXd &Pd, const Eigen::MatrixXd &Kd) override;
    const Eigen::MatrixXd& getFrictionParams() override;
    const Eigen::MatrixXd& getDepPb() override;
    const Eigen::MatrixXd& getDepPd() override;
    const Eigen::MatrixXd& getDepKd() override;

private:
    Eigen::VectorXd calculateExternalCartesianForce(const Eigen::VectorXd &q, const Eigen::VectorXd& joint_torques);
    DependencyAnalysisResult calculateDynamicParamsDependence(int sample_num, double thre);
    void shuffleVector(Eigen::VectorXd& vec, const Eigen::VectorXd& upper_limit, Eigen::VectorXd lower_limit = Eigen::VectorXd{});
    // friction helpers
    Eigen::VectorXd computeFrictionTorque(const Eigen::VectorXd &v);
    void fillFrictionRegressor(Eigen::MatrixXd &block, const Eigen::VectorXd &v);

private:
    IRobotStateProvider* state_port_{nullptr};
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::MatrixXd empty_matrix_;
    DependencyAnalysisResult dep_res_;
    Eigen::MatrixXd all_params_;
    Eigen::MatrixXd base_params_model_;
    Eigen::MatrixXd base_params_;
    Eigen::MatrixXd friction_params_;
    size_t nq_;
    bool is_ready_{false};
};
