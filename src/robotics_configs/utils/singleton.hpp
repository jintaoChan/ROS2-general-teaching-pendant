#pragma once
#include <mutex>
#include <memory>
#include <stdexcept>

template <typename T>
class Singleton {
public:
    template <typename... Args>
    static T& init(Args&&... args) {
        std::call_once(m_InitFlag, [&]() {
            m_Instance.reset(new T(std::forward<Args>(args)...));
        });
        if (!m_Instance) {
            throw std::runtime_error("Singleton already initialized with different arguments");
        }
        return *m_Instance;
    }

    static T& instance() {
        if (!m_Instance) {
            throw std::runtime_error("Singleton not initialized. Call init(...) first.");
        }
        return *m_Instance;
    }

protected:
    Singleton() = default;
    ~Singleton() = default;

    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton&) = delete;

private:
    static std::unique_ptr<T> m_Instance;
    static std::once_flag m_InitFlag;
};

template <typename T>
std::unique_ptr<T> Singleton<T>::m_Instance;

template <typename T>
std::once_flag Singleton<T>::m_InitFlag;
