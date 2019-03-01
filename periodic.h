#ifndef PERIODIC_H
#define PERIODIC_H

#include <chrono>
#include <thread>
#include <iostream>
#include <condition_variable>
#include <mutex>

template<typename T>
struct Periodic
{
    void periodicTask() { static_cast<T*>(this)->periodicTask(); }
    bool wakeFromLongSleep() { return static_cast<T*>(this)->wakeFromLongSleep(); }
    bool goToLongSleep() { return static_cast<T*>(this)->goToLongSleep(); }

    // virtual void periodicTask() = 0;
    // virtual bool wakeFromLongSleep() = 0;
    // virtual bool goToLongSleep() = 0;

    void runTask()
    {
        runPeriodicThread = 1;

        auto nextWake = std::chrono::system_clock::now();

        while(runPeriodicThread)
        {
            periodicTask();

            bool goToSleep;
            {
                std::lock_guard<std::mutex> lk(conditionMutex);
                goToSleep = this->goToLongSleep();
            }

            // poll
            if(!goToSleep && taskPeriod > 0)
            {
                nextWake += std::chrono::milliseconds{taskPeriod};
                std::this_thread::sleep_until(nextWake);

                static auto lastWokeAt = std::chrono::system_clock::now();
                auto wokeAt = std::chrono::system_clock::now();

                auto diff = wokeAt - lastWokeAt;
                // std::cout << "slept for: " << diff.count() << std::endl;
                lastWokeAt = wokeAt;
            }
            // go into long sleep
            else
            {
                std::unique_lock<std::mutex> lk(conditionMutex);
                std::cout << "Thread: " << std::this_thread::get_id() << " going into long sleep" << std::endl;
                wakeCV.wait(lk, [this]{return (!runPeriodicThread || this->wakeFromLongSleep());});
                nextWake = std::chrono::system_clock::now();
                std::cout << "Thread: " << std::this_thread::get_id() << " woke after long sleep" << std::endl;
            }
        }

        return;
    }

    void quitPeriodicTask()
    {
        {
            std::lock_guard<std::mutex> l(conditionMutex);
            runPeriodicThread = 0;
        }
        wakeCV.notify_all();
    }

protected:
    unsigned int taskPeriod;
    std::mutex conditionMutex;
    std::condition_variable wakeCV;

private:
    int runPeriodicThread;

};

#endif