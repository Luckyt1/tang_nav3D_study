#ifndef _UTILS_H
#define _UTILS_H

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <Eigen/Eigen>

#include "freedom/common_types.h"

namespace freedom{
// 计时器
class Timer{
public:
    struct TimerData
    {
        std::string name;
        std::chrono::high_resolution_clock::time_point start_time;
        double total_time = 0;
        int count = 0;
        bool is_running = false;

        TimerData(const std::string& timer_name = "") : name(timer_name) {}

        void start()
        {
            start_time = std::chrono::high_resolution_clock::now();
            is_running = true;
        }

        void stop() {
            if (!is_running) {
                return;
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            total_time += elapsed;
            ++ count;
            is_running = false;
        }

        void reset()
        {
            total_time = 0.0;
            count = 0;
            is_running = false;
        }
    };

    TimerData& operator[](const std::string& timer_name)
    {
        for (auto& timer : timers_) {
            if (timer.name == timer_name) {
                return timer;
            }
        }
        timers_.emplace_back(timer_name);
        return timers_.back();
    }

    Timer(){}
    ~Timer() = default;

private:
    std::vector<TimerData> timers_;
};

// 线程安全的vector遍历分配
// 需要保证get_ptr的指针使用周期内vector不发生改变
template <typename T>
class VectorElementGetter {
public:
    explicit VectorElementGetter(std::vector<T,Eigen::aligned_allocator<T>>& elements): elements_(elements), current_index_(0){}

    bool get_ptr(T*& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_index_ >= elements_.size())
            return false;

        element = &elements_[current_index_];
        ++ current_index_;
        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_index_ = 0;
    }

private:
    std::mutex mutex_;
    std::vector<T,Eigen::aligned_allocator<T>>& elements_;
    size_t current_index_;
};

template <typename T>
class constVectorElementGetter {
public:
    explicit constVectorElementGetter(const std::vector<T,Eigen::aligned_allocator<T>>& elements): elements_(elements), current_index_(0){}

    bool get_ptr(const T*& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_index_ >= elements_.size())
            return false;

        element = &elements_[current_index_];
        ++ current_index_;
        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_index_ = 0;
    }

private:
    std::mutex mutex_;
    const std::vector<T,Eigen::aligned_allocator<T>>& elements_;
    size_t current_index_;
};

// 默认最小idx为0,0,0且最大值相同
inline void incrementIdx(Index& idx, const unsigned int& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx)
    {
        idx.z() = 0;
        idx.y()++;

        if (idx.y() >= max_idx)
        {
            idx.y() = 0;
            idx.x()++;
        }
    }
}

// 默认最小idx为0,0,0
inline void incrementIdx(Index& idx, const Index& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx.z())
    {
        idx.z() = 0;
        idx.y()++;

        if (idx.y() >= max_idx.y())
        {
            idx.y() = 0;
            idx.x()++;
        }
    }
}

inline void incrementIdx(Index& idx, const Index& min_idx, const Index& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx.z())
    {
        idx.z() = min_idx.z();
        idx.y()++;

        if (idx.y() >= max_idx.y())
        {
            idx.y() = min_idx.y();
            idx.x()++;
        }
    }
}

class Neighbours{
public:
    Neighbours(){}
    inline void set_params(unsigned int connectivity);
    std::vector<Index> offsets;
};

inline void Neighbours::set_params(unsigned int connectivity)
{
    offsets.reserve(26);
    offsets.emplace_back(Index(-1,  0,  0));
    offsets.emplace_back(Index( 1,  0,  0));
    offsets.emplace_back(Index( 0, -1,  0));
    offsets.emplace_back(Index( 0,  1,  0));
    offsets.emplace_back(Index( 0,  0, -1));
    offsets.emplace_back(Index( 0,  0,  1));
    offsets.emplace_back(Index(-1, -1,  0));
    offsets.emplace_back(Index(-1,  1,  0));
    offsets.emplace_back(Index( 1, -1,  0));
    offsets.emplace_back(Index( 1,  1,  0));
    offsets.emplace_back(Index( 0, -1, -1));
    offsets.emplace_back(Index( 0, -1,  1));
    offsets.emplace_back(Index( 0,  1, -1));
    offsets.emplace_back(Index( 0,  1,  1));
    offsets.emplace_back(Index(-1,  0, -1));
    offsets.emplace_back(Index( 1,  0, -1));
    offsets.emplace_back(Index(-1,  0,  1));
    offsets.emplace_back(Index( 1,  0,  1));
    offsets.emplace_back(Index(-1, -1, -1));
    offsets.emplace_back(Index(-1, -1,  1));
    offsets.emplace_back(Index(-1,  1, -1));
    offsets.emplace_back(Index(-1,  1,  1));
    offsets.emplace_back(Index( 1, -1, -1));
    offsets.emplace_back(Index( 1, -1,  1));
    offsets.emplace_back(Index( 1,  1, -1));
    offsets.emplace_back(Index( 1,  1,  1));

    if(connectivity == 6 || connectivity == 18 || connectivity == 26)
        offsets.resize(connectivity);
    else
        std::cout << "Connectivity not supproted" << std::endl;
}

class ProgressBar
{
public:
    ProgressBar(std::string prefix_,int bar_length_,int total_steps_,int skip_):
        prefix(prefix_),
        bar_length(bar_length_),
        total_steps(total_steps_),
        skip(skip_),
        end(total_steps_%skip_),
        current_step(0),
        start_time(std::chrono::steady_clock::now()),
        last_print_time(std::chrono::steady_clock::now()){}
    
    inline void step()
    {
        current_step ++;

        if(!time_to_print())
            return;
        
        unsigned int cur_bar_length = (bar_length * current_step)/total_steps;
        std::string bar_string(cur_bar_length,'=');
        std::string empty_bar_string(bar_length - cur_bar_length,' ');
        
        const double elapsed = std::chrono::duration<double>(last_print_time - start_time).count();
        std::cout << std::left << std::setw(12) << prefix << "[" << bar_string << ">" << empty_bar_string << "]" << (100 * current_step)/total_steps << "% " << std::fixed << std::setprecision(3) << elapsed << "s\r";
        std::cout.flush();

        if(current_step == total_steps)
            std::cout << std::endl;
    }

    inline bool time_to_print()
    {
        auto now = std::chrono::steady_clock::now();

        if(current_step%skip == end || std::chrono::duration<double>(now - last_print_time).count() > 0.0333)
        {
            last_print_time = now;
            return true;
        }

        return false;
    }

private:
    std::string prefix;
    unsigned int bar_length;
    unsigned int total_steps;
    unsigned int current_step;
    unsigned int skip;
    unsigned int end;

    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point last_print_time;
};

}
#endif
