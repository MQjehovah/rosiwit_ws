/**
 * @file thread_pool.cpp
 * @brief FAST-LIO2 SLAM - 高性能线程池实现
 */

#include "fast_lio2_slam/common/thread_pool.h"

namespace fast_lio2_slam {

ThreadPool::ThreadPool(const ThreadPoolConfig& config)
    : config_(config), running_(false), active_threads_(0),
      total_tasks_(0), completed_tasks_(0) {
    if (config_.thread_count <= 0) {
        config_.thread_count = std::thread::hardware_concurrency();
        if (config_.thread_count <= 0) config_.thread_count = 4;
    }
    start();
}

ThreadPool::~ThreadPool() {
    stop();
}

void ThreadPool::start() {
    if (running_) return;

    running_ = true;

    // 创建工作线程
    for (int i = 0; i < config_.thread_count; ++i) {
        workers_.emplace_back([this] { workerThread(); });
    }
}

void ThreadPool::stop() {
    if (!running_) return;

    running_ = false;
    condition_.notify_all();

    // 等待所有线程结束
    for (auto& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    workers_.clear();
}

std::vector<std::future<void>> ThreadPool::submitBatch(
    const std::vector<std::function<void()>>& tasks) {

    std::vector<std::future<void>> results;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        for (const auto& task : tasks) {
            auto wrapper = std::make_shared<std::packaged_task<void()>>(task);
            results.push_back(wrapper->get_future());
            task_queue_.emplace([wrapper]() { (*wrapper)(); }, 0);
            total_tasks_++;
        }
    }

    condition_.notify_all();
    return results;
}

void ThreadPool::waitAll() {
    std::unique_lock<std::mutex> lock(wait_mutex_);

    // 等待所有任务完成
    finished_condition_.wait(lock, [this] {
        return completed_tasks_ >= total_tasks_ || !running_;
    });
}

size_t ThreadPool::getActiveThreadCount() const {
    return static_cast<size_t>(active_threads_.load());
}

size_t ThreadPool::getPendingTaskCount() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return task_queue_.size();
}

void ThreadPool::workerThread() {
    while (running_) {
        Task task;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);

            // 等待任务
            condition_.wait(lock, [this] {
                return !task_queue_.empty() || !running_;
            });

            if (!running_ && task_queue_.empty()) {
                return;
            }

            if (!task_queue_.empty()) {
                task = task_queue_.top();
                task_queue_.pop();
                active_threads_++;
            }
        }

        if (task.func) {
            // 执行任务
            try {
                task.func();
            } catch (const std::exception& e) {
                // 任务执行异常，记录但不中断线程
                // 实际应用中应该有日志记录
            }

            active_threads_--;
            completed_tasks_++;

            // 通知等待线程
            finished_condition_.notify_all();
        }
    }
}

}  // namespace fast_lio2_slam
