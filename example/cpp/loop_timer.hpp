#pragma once
#include <chrono>
#include <thread>
#include <functional>
#include <iostream>
#include <string>
#include <memory>

namespace loop
{
    /**
     * @brief A timer to ensure the frequency of thread execution.
     */
    class LoopTimer
    {
    public:
        /**
         * @brief Construct a new Timer object
         * @param period_sec time, Unit: second
         */
        explicit LoopTimer(double period_sec) : period_(period_sec) { start(); }

        double period() const { return period_; }

        /**
         * @brief Update the beginning time.
         */
        void start() { start_time_ = std::chrono::steady_clock::now(); }

        /**
         * @brief Calculate the time from the beginning to the present.
         * @return second
         */
        double elapsed_time() const
        {
            auto elapsed_duration = std::chrono::steady_clock::now() - start_time_;
            return std::chrono::duration<double>(elapsed_duration).count();
        }

        /**
         * @brief Calculate the remaining time in a period
         * @return second
         */
        double wait_time() const { return period_ - elapsed_time(); }

        /**
         * @brief Sleep for wait_time() until a period finished. If it has timeout, do nothing.
         */
        void sleep()
        {
            double wait_duration = wait_time();
            if (wait_duration > 0.0)
                std::this_thread::sleep_for(std::chrono::duration<double>(wait_duration));
            start(); // If the last one ends and then start a new timer.
        }

    private:
        double period_;
        std::chrono::steady_clock::time_point start_time_;
    };

    /**
     * @brief Maintains a thread to run once every period.
     */
    class LoopThread
    {
    public:
        /**
         * @brief Construct a new Loop object
         * @param name Indicate what the loop aims to
         * @param period time, Unit: second
         * @param callback the running function pointer
         */
        LoopThread(const std::string &name, double period, std::function<void()> callback)
            : name_(name), cb_function_(std::move(callback)), timer_(period) {}

        ~LoopThread() noexcept
        {
            try
            {
                shutdown();
            }
            catch (...)
            {
                // Suppress exceptions in destructor
            }
        }

        void start()
        {
            if (!isrunning_)
            {
                isrunning_ = true;
                thread_ = std::thread(&LoopThread::running_impl, this);
            }
        }

    private:
        void spinOnce()
        {
            timer_.start();
            ++run_times_;
            cb_function_();

            if (timer_.wait_time() > 0.0)
                timer_.sleep();
            else
                ++timeout_times_;
        }

        void running_impl()
        {
            while (isrunning_)
            {
                spinOnce();
            }
        }

        void shutdown()
        {
            if (isrunning_)
            {
                isrunning_ = false;
                if (thread_.joinable())
                    thread_.join();
            }
            std::cout << "run times of " << name_ << ":\t" << run_times_ << std::endl;
            std::cout << "timeout times of " << name_ << ":\t" << timeout_times_ << std::endl;
        }

        std::string name_{};
        bool isrunning_{false};
        LoopTimer timer_;

        std::function<void()> cb_function_;
        std::thread thread_;

        size_t run_times_{0};
        size_t timeout_times_{0};
    };
} // namespace loop