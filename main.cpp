#include <thread>

#include "PowerRune.h"

#define VIDEO_PATH "../example.mp4"

int main() {
    power_rune::PowerRune pr;
    cv::VideoCapture cap{VIDEO_PATH};
    cv::Mat image;
    while (true) {
        auto start{std::chrono::steady_clock::now()};
        if (cap.read(image) == false) {
            break;
        }

        pr.runOnce(image, 0.0, 0.0);

        auto future_time = start + std::chrono::milliseconds(1000 / power_rune::Param::FPS);
        if (std::chrono::steady_clock::now() < future_time) {
            std::this_thread::sleep_until(future_time);
        }
    }
    return 0;
}