#include <chrono>
#include <thread>
#include <iostream>

class Rate {
public:
    explicit Rate(double fps) : interval(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / fps))) {
        last_time = std::chrono::steady_clock::now();
    }

    void sleep() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - last_time;
        if (elapsed < interval) {
            std::this_thread::sleep_for(interval - elapsed);
        }
        last_time = std::chrono::steady_clock::now();
    }

private:
    std::chrono::steady_clock::duration interval;
    std::chrono::steady_clock::time_point last_time;
};