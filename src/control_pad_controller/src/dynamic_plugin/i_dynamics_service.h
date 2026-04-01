#pragma once
#include <optional>
#include <vector>
#include <utility>
#include <Eigen/Dense>
#include "robot_ports.h"

class IDynamicsService {
public:
    virtual ~IDynamicsService() = default;

    virtual void identify(const size_t& db_start_index, const size_t& db_end_index) = 0;
    virtual std::optional<JointsTorque> rnea(const JointsPosition& q, const JointsVelocity& v, const JointsAcceleration& a) = 0;
    virtual std::optional<JointsTorque> currentPoseStableTorque(const JointsPosition& q) = 0;
    virtual const bool& isReady() const = 0;

    virtual const Eigen::MatrixXd& dependenceComputation() = 0;
    virtual const Eigen::MatrixXd& getBaseParams() = 0;
    virtual void setParams(const Eigen::MatrixXd& base, const Eigen::MatrixXd& friction, const Eigen::MatrixXd& Pb, const Eigen::MatrixXd& Pd, const Eigen::MatrixXd& Kd) = 0;
    virtual const Eigen::MatrixXd& getFrictionParams() = 0;
    virtual const Eigen::MatrixXd& getDepPb() = 0;
    virtual const Eigen::MatrixXd& getDepPd() = 0;
    virtual const Eigen::MatrixXd& getDepKd() = 0;
};
