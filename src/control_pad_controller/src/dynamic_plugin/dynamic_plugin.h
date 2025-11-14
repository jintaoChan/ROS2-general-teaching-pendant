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
    void identify();
private:
    DependencyAnalysisResult calculateDynamicParamsDependence(const pinocchio::Model& model, pinocchio::Data& data, int sample_num, double thre);
    void shuffleVector(Eigen::VectorXd& vec, const Eigen::VectorXd& upper_limit, Eigen::VectorXd lower_limit = Eigen::VectorXd{});

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    DependencyAnalysisResult dep_res_;
    Eigen::MatrixXd all_params_;
    Eigen::MatrixXd base_params_;
    std::future<DependencyAnalysisResult> dep_future_;
};
