#pragma once
#include "Utility.h"

namespace power_rune {

/**
 * 0: 不显示
 * 1: 显示箭头、装甲板、中心、预测点
 * 2: 在 1 的基础上显示灯条、roi
 * 3: 在 2 的基础上显示二值化图片
 */
#define SHOW_IMAGE 1

#define CONSOLE_OUTPUT 1

struct Param {
    Param() = default;
    Param(const std::string& filename);

    void load(const std::string& filename);

    inline static float IMAGE_WIDTH, IMAGE_HEIGHT;

    const inline static cv::Scalar RED{0, 0, 255};
    const inline static cv::Scalar BLUE{255, 0, 0};
    const inline static cv::Scalar GREEN{0, 255, 0};
    const inline static cv::Scalar WHITE{255, 255, 255};
    const inline static cv::Scalar YELLOW{0, 255, 255};
    const inline static cv::Scalar PURPLE{128, 0, 128};

    inline static int FPS;

    inline static cv::Scalar DRAW_COLOR;

    inline static double ARMOR_OUTSIDE_WIDTH;
    inline static double ARMOR_INSIDE_WIDTH;
    inline static double ARMOR_INSIDE_Y;
    inline static double ARMOR_OUTSIDE_Y;
    inline static double ARMOR_OUTSIDE_HEIGHT;

    inline static const double POWER_RUNE_RADIUS{700.0};

    inline static Mode MODE;

    inline static const double GRAVITY{9.800};

    inline static double CURRENT_BULLET_SPEED;
    inline static double MIN_BULLET_SPEED;
    inline static double DEFAULT_BULLET_SPEED;

    inline static std::array<double, 3> CAMERA_TO_GIMBAL_TRANSLATION_VECTOR;
    inline static const std::array<double, 3> CAMERA_TO_GIMBAL_ROTATION_VECTOR{0.0, 0.0, 0.0};

    inline static const double ANGLE_BETWEEN_FAN_BLADES{72 * CV_PI / 180};

    inline static double COMPANSATE_TIME;
    inline static double COMPANSATE_PITCH;
    inline static double COMPANSATE_YAW;

    inline static const double SMALL_POWER_RUNE_ROTATION_SPEED{1.04719};

    inline static const double MIN_DISTANCE_TO_TARGET{4};
    inline static const double MAX_DISTANCE_TO_TARGET{10};

    inline static cv::Mat INTRINSIC_MATRIX;
    inline static cv::Mat DIST_COEFFS;

    inline static Color COLOR;

    inline static int ARROW_BRIGHTNESS_THRESHOLD;
    inline static int ARMOR_BRIGHTNESS_THRESHOLD;
    inline static const int MAX_BRIGHTNESS{255};

    inline static double LOCAL_ROI_DISTANCE_RATIO;
    inline static float LOCAL_ROI_WIDTH;

    inline static float ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD;

    inline static double GLOBAL_ROI_LENGTH_RATIO;

    inline static double MIN_ARROW_LIGHTLINE_AREA;
    inline static double MAX_ARROW_LIGHTLINE_AREA;

    inline static double MAX_ARROW_LIGHTLINE_ASPECT_RATIO;

    inline static int MIN_ARROW_LIGHTLINE_NUM;
    inline static int MAX_ARROW_LIGHTLINE_NUM;

    inline static double MIN_ARROW_ASPECT_RATIO;
    inline static double MAX_ARROW_ASPECT_RATIO;

    inline static double MAX_ARROW_AREA;

    inline static double MAX_SAME_ARROW_AREA_RATIO;

    inline static double MIN_ARMOR_LIGHTLINE_AREA;
    inline static double MAX_ARMOR_LIGHTLINE_AREA;

    inline static double MIN_ARMOR_LIGHTLINE_CONTOUR_AREA;
    inline static double MAX_ARMOR_LIGHTLINE_CONTOUR_AREA;

    inline static double MIN_ARMOR_LIGHTLINE_ASPECT_RATIO;
    inline static double MAX_ARMOR_LIGHTLINE_ASPECT_RATIO;

    inline static double MAX_SAME_ARMOR_AREA_RATIO;

    inline static double MIN_SAME_ARMOR_DISTANCE;
    inline static double MAX_SAME_ARMOR_DISTANCE;

    inline static double MIN_CENTER_AREA;
    inline static double MAX_CENTER_AREA;

    inline static double MAX_CENTER_ASPECT_RATIO;

    inline static int MIN_FIT_DATA_SIZE;
    inline static int MAX_FIT_DATA_SIZE;
};

}  // namespace power_rune