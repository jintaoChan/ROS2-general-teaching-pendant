#include "dynamic_plugin.h"
#include "robot_handle.h"

using namespace pinocchio;
using DependencyAnalysisResult = DynamicPlugin::DependencyAnalysisResult;

DynamicPlugin::DynamicPlugin()
{
    pinocchio::urdf::buildModel(RobotHandle::instance().getURDFTree(), model_);
    data_ = Data(model_);

    dep_future_ = std::async(std::launch::async,
                             &DynamicPlugin::calculateDynamicParamsDependence,
                             this,
                             std::cref(model_),
                             std::ref(data_),
                             1e4,
                             1e-5);

    all_params_ = Eigen::MatrixXd(0,1);
    for(size_t i = 1; i < model_.inertias.size(); ++i) {
        const auto& iner = model_.inertias[i];
        auto dyn_params = iner.toDynamicParameters();
        auto old_size = all_params_.size();
        all_params_.conservativeResize(old_size + dyn_params.size(), 1);
        all_params_.block(old_size, 0, dyn_params.size(), 1) = dyn_params;
    }
}

const Eigen::MatrixXd& DynamicPlugin::getBaseParams() {
    static std::once_flag base_params_initialized_flag;
    std::call_once(base_params_initialized_flag, [this](){
        dep_res_ = dep_future_.get();
        base_params_ = ((dep_res_.Pb.transpose() + dep_res_.Kd * dep_res_.Pd.transpose()) * all_params_).eval();
    });

    return base_params_;
}

DependencyAnalysisResult DynamicPlugin::calculateDynamicParamsDependence(const pinocchio::Model& model, pinocchio::Data& data, int sample_num, double thre) {
  auto joint_num = model.joints.size() - 1;
  size_t param_per_joint_num = model.inertias.front().toDynamicParameters().size();
  Eigen::VectorXd q = neutral(model);
  Eigen::VectorXd v = neutral(model);
  Eigen::VectorXd a = neutral(model);
  Eigen::VectorXd a_limit = neutral(model);
  a_limit.setOnes(); //just for dependence computation
  Eigen::MatrixXd Z(joint_num * sample_num,  joint_num * param_per_joint_num);
  for(size_t i = 0;i < sample_num;++i) {
    shuffleVector(q, model.upperPositionLimit, model.lowerPositionLimit);
    shuffleVector(v, model.velocityLimit);
    shuffleVector(a, a_limit);
    auto reg = computeJointTorqueRegressor(model,data,q,v,a);
    Z.block(i * joint_num, 0, joint_num, Z.cols()) = reg;
  }
  
  int parm_num = Z.cols();
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Z);
    
  qr.setThreshold(thre);
  int dbn = qr.rank();
  int n_dependent = parm_num - dbn; 

  Eigen::MatrixXd P = qr.colsPermutation().toDenseMatrix().cast<double>();
  Eigen::MatrixXd Pb = P.leftCols(dbn);
  Eigen::MatrixXd Pd = P.rightCols(n_dependent);

  Eigen::MatrixXd Rbd1 = qr.matrixR().topRows(parm_num);
  Eigen::MatrixXd Rb1 = Rbd1.topLeftCorner(dbn, dbn);
  Eigen::MatrixXd Rd1 = Rbd1.block(0, dbn, dbn, n_dependent);

  Eigen::MatrixXd Kd;// Rb1 * Kd = Rd1
  if (dbn > 0) {
      Kd = Rb1.triangularView<Eigen::Upper>().solve(Rd1);
  } else {
      Kd = Eigen::MatrixXd(0, n_dependent);
  }

  DependencyAnalysisResult result;
  result.Pb = Pb;
  result.Pd = Pd;
  result.Kd = Kd;
  return result;
}


void DynamicPlugin::shuffleVector(Eigen::VectorXd& vec, const Eigen::VectorXd& upper_limit, Eigen::VectorXd lower_limit) {
  std::random_device rd;
  std::mt19937 gen(rd());
  if(lower_limit.size() == Eigen::Index{0}){
    lower_limit = upper_limit * -1;
  }
  for(size_t i = 0;i < vec.size(); ++i){
    std::uniform_real_distribution<double> distrib(lower_limit(i), upper_limit(i));
    vec(i) = distrib(gen);
  }
}

Eigen::MatrixXd first_order_momentum(
    pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::MatrixXd& Pb,
    const Eigen::MatrixXd& base_params,
    const Eigen::MatrixXd& K0, //diagonal matrix
    const double& frequency,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const Eigen::VectorXd& t
    ) {
    model.gravity.linear().setZero();
    static auto zero = Eigen::VectorXd(q.size()).setZero();
    static auto r = zero;
    static auto sum = zero;
    static auto p0 = (pinocchio::computeJointTorqueRegressor(model, data, q, zero, v) * Pb * base_params).eval();
    auto Mv = (pinocchio::computeJointTorqueRegressor(model, data, q, zero, v) * Pb * base_params).eval();
    auto C = (pinocchio::computeJointTorqueRegressor(model, data, q, v, zero) * Pb * base_params).eval();
    model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);
    auto G = (pinocchio::computeJointTorqueRegressor(model, data, q, zero, zero) * Pb * base_params).eval();
    auto inter = (t + C - G + r) * 1/frequency;
    sum += inter;
    r = K0 * (Mv - sum - p0);

    return r.eval();
}
