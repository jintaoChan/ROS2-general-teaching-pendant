#pragma once

#include <QWidget>
#include <optional>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <Eigen/Eigen>
#include <cereal/archives/binary.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/vector.hpp>
#include "simulate_external_force_bar.h"

class ParamIdentification : public QWidget
{
    Q_OBJECT

public:
    explicit ParamIdentification(QWidget *parent = nullptr);
    ~ParamIdentification();

private:
    std::optional<trajectory_msgs::msg::JointTrajectory> loadTrajFile(const QString& path);

signals:
    void identifyFinished();

private slots:
    void saveInfoClicked();
    void loadParamClicked();
    void identifyClicked();

private:
    QPushButton* copy_info_button_;
    QPushButton* load_param_button_;
    QPushButton* identify_button_;
    QLabel* base_param_size_title_;
    QLabel* base_param_size_;
    QHBoxLayout* base_param_size_layout_;
    QVBoxLayout* layout_;
    QList<SimulateExternalForceBar*> sefb_list_;
};

namespace cereal {
        
    namespace eigen_internal {
        template <class Archive, typename Derived>
        typename std::enable_if<(Derived::RowsAtCompileTime == Eigen::Dynamic || Derived::ColsAtCompileTime == Eigen::Dynamic), void>::type
        load_dimensions(Archive& archive, Derived& m, Eigen::Index& rows, Eigen::Index& cols) {
            archive(CEREAL_NVP(rows), CEREAL_NVP(cols));
        }

        template <class Archive, typename Derived>
        typename std::enable_if<(Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime != Eigen::Dynamic), void>::type
        load_dimensions(Archive& archive, Derived& m, Eigen::Index& rows, Eigen::Index& cols) {
        }
    }

    namespace eigen_internal {
        template <class Archive, typename Derived>
        typename std::enable_if<(Derived::RowsAtCompileTime == Eigen::Dynamic || Derived::ColsAtCompileTime == Eigen::Dynamic), void>::type
        serialize_dimensions(Archive& archive, const Derived& m) {
            const Eigen::Index rows = m.rows();
            const Eigen::Index cols = m.cols();
            archive(CEREAL_NVP(rows), CEREAL_NVP(cols));
        }

        template <class Archive, typename Derived>
        typename std::enable_if<(Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime != Eigen::Dynamic), void>::type
        serialize_dimensions(Archive& archive, const Derived& m) {
        }
    }

    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void save(Archive& archive, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m) {
        eigen_internal::serialize_dimensions(archive, m);
        std::vector<_Scalar> vec(m.data(), m.data() + m.size());
        archive(CEREAL_NVP(vec));
    }


    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void load(Archive& archive, 
              Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m) {
        
        Eigen::Index rows = _Rows;
        Eigen::Index cols = _Cols;
        
        eigen_internal::load_dimensions(archive, m, rows, cols);

        if (m.rows() != rows || m.cols() != cols) {
            m.resize(rows, cols);
        }
        
        std::vector<_Scalar> vec;
        archive(CEREAL_NVP(vec)); 
        
        if (m.rows() * m.cols() != vec.size()) {
            m.resize(rows, cols);
        }
        std::copy(vec.begin(), vec.end(), m.data());
    }

}
