#pragma once
#include <mutex>
#include <memory>
#include <stdexcept>

template <typename T>
class Singleton {
public:
    template <typename... Args>
    static T& init(Args&&... args) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        if (!m_Instance) {
            m_Instance.reset(new T(std::forward<Args>(args)...));
        }
        return *m_Instance;
    }

    static T& instance() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        if (!m_Instance) {
            throw std::runtime_error("Singleton not initialized. Call init(...) first.");
        }
        return *m_Instance;
    }

    static void destroy() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        m_Instance.reset();
    }

protected:
    Singleton() = default;
    ~Singleton() = default;

    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton&) = delete;

private:
    static std::unique_ptr<T> m_Instance;
    static std::mutex m_Mutex;
};

template <typename T>
std::unique_ptr<T> Singleton<T>::m_Instance;

template <typename T>
std::mutex Singleton<T>::m_Mutex;
