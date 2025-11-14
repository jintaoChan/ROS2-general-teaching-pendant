#pragma once
#include <vector>
#include <unordered_map>
#include <QList>
#include <QPointF>
#include <magic_enum/magic_enum.hpp>
#include "singleton.hpp"


enum class DataTypeEnum : char {
    POSITION = 0,
    VELOCITY,
    ACCELERATION,
    TORQUE,
    ESTIMATED_TORQUE,
};


class DataType {

public:
    DataType(const size_t& buffer_size);
    ~DataType();
    DataType(const DataType& other);
    DataType& operator=(const DataType& other);
    DataType(DataType&& other) noexcept = default;
    DataType& operator=(DataType&& other) noexcept = default;

    void appendData(const double& d);
    QList<QPointF> getSnapShot(size_t start, size_t n);
    QList<QPointF> getSnapShot(size_t n);
    const double& getMax() const;
    const double& getMin() const;
    size_t getCurrentSize() const;
    double getElement(size_t idx) const;
    double getElementByNow(size_t idx) const;

    class Impl;
    std::unique_ptr<Impl> impl_;
};

class DataBase : public Singleton<DataBase> {
    friend class Singleton<DataBase>;

public:
    // the unit of control_period is milisecond
    DataBase(size_t buffer_size, const std::vector<std::string>& joints_names, const uint64_t& control_period);
    ~DataBase();


    void appendData(DataTypeEnum type, std::string joint_name, double d);
    const DataType& getData(DataTypeEnum type, std::string joint_name) const;
    std::string toPlainText() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
