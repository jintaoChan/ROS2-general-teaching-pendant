#include "dynamic_plugin.h"
#include "robot_handle.h"
#include "database.h"
#include "functional.hpp"

#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/jacobian.hpp>


using namespace pinocchio;
using DependencyAnalysisResult = DynamicPlugin::DependencyAnalysisResult;

DynamicPlugin::DynamicPlugin()
{
    auto urdf_string = AcquireParam<std::string>("/robot_state_publisher", "robot_description").value();
    auto urdf_tree = ::urdf::parseURDF(urdf_string);

    pinocchio::urdf::buildModel(urdf_tree, model_);
    data_ = Data(model_);

    dep_future_ = std::async(std::launch::async,
                             &DynamicPlugin::calculateDynamicParamsDependence,
                             this,
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
    auto n = model_.inertias.size() - 1;
    K0_ = Eigen::MatrixXd::Identity(n, n) * 10;

}

const Eigen::MatrixXd& DynamicPlugin::getBaseParams() {
    static std::once_flag base_params_model_initialized_flag;
    std::call_once(base_params_model_initialized_flag, [this](){
        dep_res_ = dep_future_.get();
        base_params_model_ = ((dep_res_.Pb.transpose() + dep_res_.Kd * dep_res_.Pd.transpose()) * all_params_).eval();
    });

    return base_params_model_;
}

void DynamicPlugin::identify(const size_t &db_start_index, const size_t &db_end_index)
{
    getBaseParams();
    Eigen::MatrixXd q, v, a, t;
    auto n = RobotHandle::instance().getJointNums();
    auto db = DataBase::instance().getAllData();
    for(const auto& name : RobotHandle::instance().getJointsName()) {
        auto& joint = db.at(name);
        auto q_list = joint.at(DataTypeEnum::POSITION).getSnapShot(db_start_index, db_end_index);
        auto v_list = joint.at(DataTypeEnum::VELOCITY).getSnapShot(db_start_index, db_end_index);
        auto a_list = joint.at(DataTypeEnum::ACCELERATION).getSnapShot(db_start_index, db_end_index);
        auto t_list = joint.at(DataTypeEnum::TORQUE).getSnapShot(db_start_index, db_end_index);
        auto length = q_list.size();
        Eigen::VectorXd q_row(length), v_row(length), a_row(length), t_row(length);
        for(size_t i = 0 ;i < q_list.size(); ++i) {
            q_row(i) = q_list[i].y();
            v_row(i) = v_list[i].y();
            a_row(i) = a_list[i].y();
            t_row(i) = t_list[i].y();
        }
        q.conservativeResize(q.rows() + 1, q_row.size());
        q.row(q.rows() - 1) = q_row;
        v.conservativeResize(v.rows() + 1, v_row.size());
        v.row(v.rows() - 1) = v_row;
        a.conservativeResize(a.rows() + 1, a_row.size());
        a.row(a.rows() - 1) = a_row;
        t.conservativeResize(t.rows() + 1, t_row.size());
        t.row(t.rows() - 1) = t_row;
    }
    Eigen::MatrixXd tau_b(0,1);
    for(size_t i = 0; i < t.cols(); ++i) {
        auto old_size = tau_b.size();
        tau_b.conservativeResize(old_size + t.col(i).size(), 1);
        tau_b.block(old_size, 0, t.col(i).size(), 1) = t.col(i);
    }
    Eigen::MatrixXd Z(n * q.cols(),  dep_res_.Pb.cols());
    Eigen::MatrixXd tau_rnea(n * q.cols(), 1);
    Eigen::MatrixXd ext_tor(n * q.cols(), 1);
    auto tau_for_observer_test = t;
    for(size_t i = 0.4 * t.cols(); i < 0.6 * t.cols(); ++i) {
        tau_for_observer_test.col(i).array() += 5;
    }
    for(size_t i = 0; i < q.cols(); ++i) {
        auto H = pinocchio::computeJointTorqueRegressor(model_, data_, q.col(i), v.col(i), a.col(i));
        auto Hb = H * dep_res_.Pb;
        Z.block(i * n, 0, n, Z.cols()) = Hb;
        auto tau = pinocchio::rnea(model_, data_, q.col(i), v.col(i), a.col(i));
        tau_rnea.block(i * n, 0, n, 1) = tau;
    }
    base_params_ = ((Z.transpose() * Z).inverse() * Z.transpose() * tau_b).eval();
    is_ready_ = true;
}

std::pair<std::vector<double>, std::vector<double>>  DynamicPlugin::firstOrderMomentum(const std::vector<double>& q, const std::vector<double>& v, const std::vector<double>& t)
{
    Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    Eigen::VectorXd v_eigen = Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
    Eigen::VectorXd t_eigen = Eigen::Map<const Eigen::VectorXd>(t.data(), t.size());
    auto res = firstOrderMomentum(q_eigen, v_eigen, t_eigen);
    auto joint_torques = std::vector<double>(res.first.data(), res.first.data() + res.first.size());
    auto cartesian_force = std::vector<double>(res.second.data(), res.second.data() + res.second.size());
    return {joint_torques, cartesian_force};
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> DynamicPlugin::firstOrderMomentum(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const Eigen::VectorXd& t
    ) {
    const static auto& peroid = RobotHandle::instance().getControllerUpdatePeriod() / 1e3;
    model_.gravity.linear().setZero();
    const static auto zero = Eigen::VectorXd(q.size()).setZero();
    static auto r = zero;
    static auto sum = zero;
    static auto p0 = (pinocchio::computeJointTorqueRegressor(model_, data_, q, zero, v) * dep_res_.Pb * base_params_).eval();
    auto Mv = (pinocchio::computeJointTorqueRegressor(model_, data_, q, zero, v) * dep_res_.Pb * base_params_).eval();
    auto C = (pinocchio::computeJointTorqueRegressor(model_, data_, q, v, zero) * dep_res_.Pb * base_params_).eval();
    model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);
    auto G = (pinocchio::computeJointTorqueRegressor(model_, data_, q, zero, zero) * dep_res_.Pb * base_params_).eval();
    auto inter = (t + C - G + r) * peroid;
    sum += inter;
    r = K0_ * (Mv - sum - p0);

    auto joint_torques = r.eval().reshaped();
    auto cartesian_force = calculateExternalCartesianForce(q, joint_torques);
    return {joint_torques, cartesian_force};
}

const bool &DynamicPlugin::isReady() const
{
    return is_ready_;
}

Eigen::VectorXd DynamicPlugin::calculateExternalCartesianForce(const Eigen::VectorXd &q, const Eigen::VectorXd &joint_torques)
{
    pinocchio::forwardKinematics(model_, data_, q);
    auto joc = pinocchio::computeJointJacobians(model_, data_, q);
    Eigen::MatrixXd JJt = joc * joc.transpose();
    double lambda = 1e-6;
    JJt += lambda * Eigen::MatrixXd::Identity(6,6);

    Eigen::VectorXd rhs = joc * joint_torques;

    Eigen::VectorXd f;
    Eigen::LDLT<Eigen::MatrixXd> solver(JJt);
    if (solver.info() == Eigen::Success)
    {
        f = solver.solve(rhs);
    }
    else
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(joc.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd Jt_pinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
        f = joc * Jt_pinv * joint_torques;
    }
    return f.eval();
}

DependencyAnalysisResult DynamicPlugin::calculateDynamicParamsDependence(int sample_num, double thre) {
  auto joint_num = model_.joints.size() - 1;
  size_t param_per_joint_num = model_.inertias.front().toDynamicParameters().size();
  Eigen::VectorXd q = neutral(model_);
  Eigen::VectorXd v = neutral(model_);
  Eigen::VectorXd a = neutral(model_);
  Eigen::VectorXd a_limit = neutral(model_);
  a_limit.setOnes(); //just for dependence computation
  Eigen::MatrixXd Z(joint_num * sample_num,  joint_num * param_per_joint_num);
  for(size_t i = 0;i < sample_num;++i) {
    shuffleVector(q, model_.upperPositionLimit, model_.lowerPositionLimit);
    shuffleVector(v, model_.velocityLimit);
    shuffleVector(a, a_limit);
    auto reg = computeJointTorqueRegressor(model_,data_,q,v,a);
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
