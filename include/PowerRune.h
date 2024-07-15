#pragma once

#include "Calculator.h"
#include "Detector.h"
#include "Param.h"
#include "Utility.h"

#define CONFIG_PATH "../config.yaml"

namespace power_rune {

class PowerRune {
   public:
    PowerRune();
    bool runOnce(const cv::Mat& image, double pitch, double yaw, double roll = 0.0);

   private:
    Param m_param;
    Detector m_detector;
    Calculator m_calculator;
};

}  // namespace power_rune
