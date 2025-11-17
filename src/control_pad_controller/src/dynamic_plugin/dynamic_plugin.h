#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/regressor.hpp>
#include <future>
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
    const Eigen::MatrixXd& getBaseParams();
    void identify(const size_t& db_start_index, const size_t& db_end_index);
    std::pair<std::vector<double>, std::vector<double>> firstOrderMomentum(const std::vector<double>& q, const std::vector<double>& v, const std::vector<double>& t);
    std::pair<Eigen::VectorXd, Eigen::VectorXd> firstOrderMomentum(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& t);
    const bool& isReady() const;
private:
    Eigen::VectorXd calculateExternalCartesianForce(const Eigen::VectorXd &q, const Eigen::VectorXd& joint_torques);
    DependencyAnalysisResult calculateDynamicParamsDependence(int sample_num, double thre);
    void shuffleVector(Eigen::VectorXd& vec, const Eigen::VectorXd& upper_limit, Eigen::VectorXd lower_limit = Eigen::VectorXd{});

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    DependencyAnalysisResult dep_res_;
    Eigen::MatrixXd all_params_;
    Eigen::MatrixXd base_params_model_;
    Eigen::MatrixXd base_params_;
    Eigen::MatrixXd K0_;// first order momentum observer gains
    std::future<DependencyAnalysisResult> dep_future_;
    bool is_ready_{false};
};
