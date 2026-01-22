#include "database.h"

class DataType::Impl {
public:
    Impl(const size_t& buffer_size, const uint64_t& update_period) :
        head_index_{0},
        buffer_size_{buffer_size},
        update_period_{update_period},
        is_full_{false}
    {
        data_base_.resize(buffer_size, 0.0);
    }
public:
    std::vector<double> data_base_;
    size_t head_index_;
    size_t buffer_size_;
    uint64_t update_period_;
    bool is_full_;
    double max_{std::numeric_limits<double>::lowest()};
    double min_{std::numeric_limits<double>::max()};
    double idx_{0};
};

DataType::DataType(const size_t &buffer_size, const uint64_t& update_period)
    : impl_(std::make_unique<Impl>(buffer_size, update_period))
{
}

DataType::~DataType() = default;

DataType::DataType(const DataType &other)
    : impl_(std::make_unique<Impl>(*other.impl_))
{}

DataType& DataType::operator=(const DataType& other)
{
    if (this == &other)
        return *this;

    impl_ = std::make_unique<Impl>(*other.impl_);
    return *this;
}

void DataType::appendData(const double &d)
{
    impl_->data_base_[impl_->head_index_] = d;
    impl_->max_ = std::max(d, impl_->max_);
    impl_->min_ = std::min(d, impl_->min_);
    impl_->head_index_ = (impl_->head_index_ + 1) % impl_->buffer_size_;

    if (!impl_->is_full_ && impl_->head_index_ == 0)
    {
        impl_->is_full_ = true;
    }
}

void DataType::clear()
{
    *this = DataType(impl_->buffer_size_, impl_->update_period_);
}

std::vector<double> DataType::getSnapShot(size_t start, size_t n) const {
    std::vector<double> snapshot;
    if(start >=impl_-> head_index_ && impl_->is_full_){
        n = (start + n > impl_->head_index_ + impl_->buffer_size_) ? (impl_->head_index_ + impl_->buffer_size_ - start - 1) : n;
        if(start + n > impl_->buffer_size_){
            size_t count = impl_->buffer_size_ - start;
            snapshot.insert(snapshot.end(), impl_->data_base_.begin() + start, impl_->data_base_.begin() + start + count);
            snapshot.insert(snapshot.end(), impl_->data_base_.begin(), impl_->data_base_.begin() + count);
        }
        else{
            snapshot.insert(snapshot.end(), impl_->data_base_.begin() + start, impl_->data_base_.begin() + start + n);
        }
    }
    else if (start < impl_->head_index_) {
        n = (start + n > impl_->head_index_) ? (impl_->head_index_ - start - 1) : n;
        snapshot.insert(snapshot.end(), impl_->data_base_.begin() + start, impl_->data_base_.begin() + start + n);
    }
    return snapshot;
}

std::vector<double> DataType::getSnapShot(size_t n) const {
    if(!impl_->is_full_){
        if(n > impl_->head_index_) {
            return getSnapShot(0, impl_->head_index_);
        }
        else {
            return getSnapShot(impl_->head_index_ - n, n);
        }
    }
    else{
        return getSnapShot((impl_->head_index_ + impl_->buffer_size_ - n)% impl_->buffer_size_, n);
    }
}

const double& DataType::getMax() const {
    return impl_->max_;
}

const double& DataType::getMin() const {
    return impl_->min_;
}

size_t DataType::getCurrentSize() const {
    return  impl_->is_full_ ? impl_->buffer_size_ : impl_->head_index_;
}

double DataType::getElement(size_t idx) const {
    return impl_->data_base_.at(idx);
}

double DataType::getElementByNow(size_t idx) const {
    return getElement((impl_->buffer_size_ + impl_->head_index_ - idx) % impl_->buffer_size_);
}

class DataBase::Impl {
public:
    Impl(size_t buffer_size, const std::vector<std::string>& joints_names, const uint64_t& control_period)
    :
        buffer_size_(buffer_size),
        joints_names_(joints_names),
        control_period_(control_period)
    {

    }
public:
    std::unordered_map<std::string, std::unordered_map<DataTypeEnum, DataType>> data_base_;
    size_t buffer_size_;
    std::vector<std::string> joints_names_;
    uint64_t control_period_;
};

DataBase::DataBase(size_t buffer_size, const std::vector<std::string>& joints_names, const uint64_t& control_period)
    : 
    impl_(std::make_unique<Impl>(buffer_size, joints_names, control_period))
{
    auto list = magic_enum::enum_values<DataTypeEnum>();
    for(const auto& n: joints_names) {
        std::unordered_map<DataTypeEnum, DataType> type_layer;
        for(const auto& e: list) {
            type_layer.emplace(e, DataType(impl_->buffer_size_, impl_->control_period_));
        }
        impl_->data_base_.emplace(n, type_layer);
    }
}

DataBase::~DataBase() = default;

void DataBase::appendData(DataTypeEnum type, std::string joint_name, double d) {
    static bool is_first = false;
    if(type == DataTypeEnum::ACCELERATION) {return;}
    impl_->data_base_.at(joint_name).at(type).appendData(d);
    if(type == DataTypeEnum::VELOCITY) {
        if(is_first) {
            impl_->data_base_.at(joint_name).at(DataTypeEnum::ACCELERATION).appendData(d / ((double)impl_->control_period_ / 1e9));
            is_first = false;
        }
        else {
            impl_->data_base_.at(joint_name).at(DataTypeEnum::ACCELERATION).appendData((d - impl_->data_base_.at(joint_name).at(DataTypeEnum::VELOCITY).getElementByNow(2)) / (2 * (double)impl_->control_period_ / 1e9));
        }
    }
}

void DataBase::clear()
{
    for(auto& i : impl_->data_base_) {
        for(auto& j : i.second) {
            j.second.clear();
        }
    }
}

std::unordered_map<std::string, std::unordered_map<DataTypeEnum, DataType>> DataBase::getAllData() const {
    return impl_->data_base_;
}

const DataType& DataBase::getData(DataTypeEnum type, std::string joint_name) const {
    return impl_->data_base_.at(joint_name).at(type);
}

std::string DataBase::toPlainText() const {
    auto copy = impl_->data_base_;
    std::string res;
    for(const auto& j : impl_->joints_names_) {
        for(const auto& d : copy[j]) {
            res.append(j);
            res.append("-");
            res.append(std::string(magic_enum::enum_name(d.first)));
            res.append("\t");
        }
    }
    res.append("\n");
    size_t id{0};
    while(id < impl_->buffer_size_) {
        for(const auto& j : impl_->joints_names_) {
            for(const auto& d : copy[j]) {
                res.append(std::to_string(d.second.getElement(id)));
                res.append("\t");
            }
        }
        res.append("\n");
        ++id;
    }
    return res;
}

size_t DataBase::getCurrentIndex() const
{
    return impl_->data_base_.begin()->second.at(DataTypeEnum::POSITION).impl_->head_index_;
}

size_t DataBase::getSize() const
{
    return impl_->buffer_size_;
}

