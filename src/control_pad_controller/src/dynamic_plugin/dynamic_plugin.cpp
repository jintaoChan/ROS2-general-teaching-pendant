#include "dynamic_plugin.h"
#include "database.h"
#include "functional.hpp"

#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/jacobian.hpp>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>


using namespace pinocchio;
using DependencyAnalysisResult = DynamicPlugin::DependencyAnalysisResult;

DynamicPlugin::DynamicPlugin()
{
    auto urdf_string = AcquireParam<std::string>("/robot_state_publisher", "robot_description").value();
    auto urdf_tree = ::urdf::parseURDF(urdf_string);

    pinocchio::urdf::buildModel(urdf_tree, model_);
    data_ = Data(model_);

    all_params_ = Eigen::MatrixXd(0,1);
    for(size_t i = 1; i < model_.inertias.size(); ++i) {
        const auto& iner = model_.inertias[i];
        auto dyn_params = iner.toDynamicParameters();
        auto old_size = all_params_.size();
        all_params_.conservativeResize(old_size + dyn_params.size(), 1);
        all_params_.block(old_size, 0, dyn_params.size(), 1) = dyn_params;
    }
    nq_ = model_.inertias.size() - 1;
    K0_ = Eigen::MatrixXd::Identity(nq_, nq_) * 10;

}

void DynamicPlugin::identify(const size_t &db_start_index, const size_t &db_end_index)
{
    using clock = std::chrono::high_resolution_clock;
    using ms = std::chrono::duration<double, std::milli>;

    auto t_start = clock::now();

    dependenceComputation();
    Eigen::MatrixXd q, v, a, t;
    auto n = RobotHandle::instance().getJointNums();
    auto db = DataBase::instance().getAllData();

    // determine sample count (use minimum across joints to be safe)
    size_t sample_count = std::numeric_limits<size_t>::max();
    for (const auto &name : RobotHandle::instance().getJointsName()) {
        auto &joint = db.at(name);
        auto q_list = joint.at(DataTypeEnum::POSITION).getSnapShot(db_start_index, db_end_index - db_start_index);
        sample_count = std::min(sample_count, (size_t)q_list.size());
    }
    if (sample_count == std::numeric_limits<size_t>::max() || sample_count == 0) {
        RCLCPP_WARN(rclcpp::get_logger("DynamicPlugin"), "No samples available for identification");
        return;
    }

    auto t_data_gather_start = clock::now();
    // preallocate matrices: rows = joints, cols = samples
    q = Eigen::MatrixXd(n, sample_count);
    v = Eigen::MatrixXd(n, sample_count);
    a = Eigen::MatrixXd(n, sample_count);
    t = Eigen::MatrixXd(n, sample_count);

    size_t row = 0;
    for (const auto &name : RobotHandle::instance().getJointsName()) {
        auto &joint = db.at(name);
        auto q_list = joint.at(DataTypeEnum::POSITION).getSnapShot(db_start_index, db_end_index);
        auto v_list = joint.at(DataTypeEnum::VELOCITY).getSnapShot(db_start_index, db_end_index);
        auto a_list = joint.at(DataTypeEnum::ACCELERATION).getSnapShot(db_start_index, db_end_index);
        auto t_list = joint.at(DataTypeEnum::TORQUE).getSnapShot(db_start_index, db_end_index);
        // fill row (truncate to sample_count if needed)
        for (size_t i = 0; i < sample_count; ++i) {
            q(row, i) = q_list[i];
            v(row, i) = v_list[i];
            a(row, i) = a_list[i];
            t(row, i) = t_list[i];
        }
        ++row;
    }
    auto t_data_gather_end = clock::now();
    std::cout << "[DynamicPlugin::identify] Data gather: " << std::fixed << std::setprecision(2)
              << ms(t_data_gather_end - t_data_gather_start).count() << " ms" << std::endl;

    // Estimate dynamic base parameters and friction parameters together
    Eigen::MatrixXd tau_b(n * sample_count, 1);
    int dyn_cols = static_cast<int>(dep_res_.Pb.cols());
    int friction_cols = 3 * static_cast<int>(n);
    Eigen::MatrixXd Z_ext(n * sample_count, dyn_cols + friction_cols);

    auto t_build_Zext_start = clock::now();
    for (size_t i = 0; i < sample_count; ++i) {
        tau_b.block(i * n, 0, n, 1) = t.col(i);
        auto q_col = q.col(i);
        auto v_col = v.col(i);
        auto a_col = a.col(i);
        auto H = pinocchio::computeJointTorqueRegressor(model_, data_, q_col, v_col, a_col);
        auto Hb = H * dep_res_.Pb;
        Z_ext.block(i * n, 0, n, dyn_cols) = Hb;
        Eigen::MatrixXd friction_block = Eigen::MatrixXd::Zero(n, friction_cols);
        fillFrictionRegressor(friction_block, v_col);
        Z_ext.block(i * n, dyn_cols, n, friction_cols) = friction_block;
    }
    auto t_build_Zext_end = clock::now();
    std::cout << "[DynamicPlugin::identify] Build Z_ext: " << std::fixed << std::setprecision(2)
              << ms(t_build_Zext_end - t_build_Zext_start).count() << " ms" << std::endl;

    auto t_qr_start = clock::now();
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Z_ext);
    Eigen::VectorXd param = qr.solve(tau_b);
    auto t_qr_end = clock::now();
    std::cout << "[DynamicPlugin::identify] QR solve: " << std::fixed << std::setprecision(2)
              << ms(t_qr_end - t_qr_start).count() << " ms" << std::endl;

    base_params_ = param.topRows(dyn_cols);
    friction_params_ = param.bottomRows(friction_cols);
    // mark ready
    is_ready_ = true;
    auto t_end = clock::now();
    std::cout << "[DynamicPlugin::identify] Total time: " << std::fixed << std::setprecision(2)
              << ms(t_end - t_start).count() << " ms" << std::endl;
}

std::optional<JointsTorque> DynamicPlugin::rnea(const JointsPosition &q, const JointsVelocity &v, const JointsAcceleration &a)
{
    try {
        JointsTorque res;
        static const auto& joint_names = RobotHandle::instance().getJointsName();
        static const auto& drag_params = RobotHandle::instance().getDragParams();
        static const auto& external_force = RobotHandle::instance().getCurrentJointEstimatedExternalTorque();
        Eigen::VectorXd eq(nq_), ev(nq_), ea(nq_), eef(nq_);

        for(size_t i = 0; i < nq_; ++i) {
            eq(i) = q.at(joint_names[i]).joint_value;
            ev(i) = v.at(joint_names[i]).joint_value;
            ea(i) = a.at(joint_names[i]).joint_value;
            eef(i) = external_force.at(joint_names[i]).joint_value;
        }
        auto et = pinocchio::computeJointTorqueRegressor(model_, data_, eq, ev, ea);
        et = (et * dep_res_.Pb * base_params_).eval();
        auto t = (et +
                  computeFrictionTorque(ev) +
                  -drag_params.at(DragParamEnum::D).cwiseProduct(ev) +
                  drag_params.at(DragParamEnum::M).cwiseProduct(eef)).eval();
        for(size_t i = 0; i < nq_; ++i) {
            res[joint_names[i]].joint_value = t(i);
        }
        return res;
    }
    catch(std::exception& e){
        return std::nullopt;
    }
}

std::optional<JointsTorque> DynamicPlugin::currentPoseStableTorque(const JointsPosition &q)
{
    static std::once_flag initialize_zero_flag;
    static JointsAcceleration zero;
    std::call_once(initialize_zero_flag, [&](){
        for(const auto& n : RobotHandle::instance().getJointsName()) {
            zero[n].joint_value = 0;
        }
    });
    return DynamicPlugin::instance().rnea(RobotHandle::instance().getCurrentJointPosition(), zero, zero);
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
    const static auto& peroid = RobotHandle::instance().getControllerUpdatePeriod() / 1e9;
    model_.gravity.linear().setZero();
    const static auto zero = Eigen::VectorXd(q.size()).setZero();
    static auto r = zero;
    static auto sum = zero;
    static auto p0 = (pinocchio::computeJointTorqueRegressor(model_, data_, q, zero, v) * dep_res_.Pb * base_params_).eval();
    auto Mv = (pinocchio::computeJointTorqueRegressor(model_, data_, q, zero, v) * dep_res_.Pb * base_params_).eval();
    auto C = (pinocchio::computeJointTorqueRegressor(model_, data_, q, v, zero) * dep_res_.Pb * base_params_).eval();
    model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);
    auto G = (pinocchio::computeJointTorqueRegressor(model_, data_, q, zero, zero) * dep_res_.Pb * base_params_).eval();
    // compute friction torque using estimated friction parameters (if available)
    Eigen::VectorXd tau_f = Eigen::VectorXd::Zero(q.size());
    tau_f = computeFrictionTorque(v);

    auto inter = (t - tau_f + C - G + r) * peroid;
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

const Eigen::MatrixXd& DynamicPlugin::dependenceComputation() {
    calculateDynamicParamsDependence(1e4, 1e-5);
    base_params_model_ = ((dep_res_.Pb.transpose() + dep_res_.Kd * dep_res_.Pd.transpose()) * all_params_).eval();

    return base_params_model_;
}

const Eigen::MatrixXd &DynamicPlugin::getBaseParams()
{
    if(!is_ready_) {
        std::cout << "No params available" << std::endl;
        return Eigen::MatrixXd{};
    }
    return base_params_;
}

void DynamicPlugin::setParams(const Eigen::MatrixXd &base, const Eigen::MatrixXd &friction, const Eigen::MatrixXd &Pb, const Eigen::MatrixXd &Pd, const Eigen::MatrixXd &Kd)
{
    base_params_ = base;
    friction_params_ = friction;
    dep_res_.Pb = Pb;
    dep_res_.Pd = Pd;
    dep_res_.Kd = Kd;
    is_ready_ = true;
}

const Eigen::MatrixXd &DynamicPlugin::getFrictionParams()
{
    if(!is_ready_) {
        std::cout << "No params available" << std::endl;
        return Eigen::MatrixXd{};
    }
    return friction_params_;
}

const Eigen::MatrixXd &DynamicPlugin::getDepPb()
{
    if(!is_ready_) {
        std::cout << "No params available" << std::endl;
        return Eigen::MatrixXd{};
    }
    return dep_res_.Pb;
}

const Eigen::MatrixXd &DynamicPlugin::getDepPd()
{
    if(!is_ready_) {
        std::cout << "No params available" << std::endl;
        return Eigen::MatrixXd{};
    }
    return dep_res_.Pd;
}

const Eigen::MatrixXd &DynamicPlugin::getDepKd()
{
    if(!is_ready_) {
        std::cout << "No params available" << std::endl;
        return Eigen::MatrixXd{};
    }
    return dep_res_.Kd;
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

static inline double signum(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

Eigen::VectorXd DynamicPlugin::computeFrictionTorque(const Eigen::VectorXd &v) {
    Eigen::VectorXd tau_f = Eigen::VectorXd::Zero(nq_);
    for (size_t j = 0; j < nq_; ++j) {
        double vj = v(j);
        double c = 0.0, kv = 0.0, b = 0.0;
        c = friction_params_(3 * j + 0);
        kv = friction_params_(3 * j + 1);
        b = friction_params_(3 * j + 2);
        tau_f(j) = c * signum(vj) + kv * vj + b;
    }
    return tau_f;
}

void DynamicPlugin::fillFrictionRegressor(Eigen::MatrixXd &block, const Eigen::VectorXd &v) {
    int nj = static_cast<int>(v.size());
    block.setZero();
    for (int j = 0; j < nj; ++j) {
        double vj = v(j);
        block(j, 3*j) = signum(vj);
        block(j, 3*j + 1) = vj;
        block(j, 3*j + 2) = 1.0;
    }
}
