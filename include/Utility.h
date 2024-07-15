#pragma once

#include <opencv2/opencv.hpp>

namespace power_rune {

enum class Direction { UNKNOWN, STABLE, ANTI_CLOCKWISE, CLOCKWISE };  // 旋转方向
enum class Convexity { UNKNOWN, CONCAVE, CONVEX };                    // 拟合曲线凹凸性
enum class Mode { SMALL, BIG };                                       // 模式：小符，大符
enum class Color { RED, BLUE };                                       // 颜色
enum class Status {
    SUCCESS,
    ARROW_FAILURE,
    ARMOR_FAILURE,
    CENTER_FAILURE
};  // 成功，箭头检测失败，装甲板检测失败，中心R检测失败

struct Frame {
    Frame() = default;
    Frame(const cv::Mat& image, const std::chrono::steady_clock::time_point& time, double pitch, double yaw,
          double roll)
        : m_image{image}, m_time{time}, m_roll{roll}, m_pitch{pitch}, m_yaw{yaw} {}
    cv::Mat m_image;
    std::chrono::steady_clock::time_point m_time;
    double m_roll, m_pitch, m_yaw;
    void set(const cv::Mat& image, const std::chrono::steady_clock::time_point& time, double pitch,
             double yaw, double roll) {
        m_image = image;
        m_time = time;
        m_roll = roll;
        m_pitch = pitch;
        m_yaw = yaw;
    }
};

/**
 * @brief 两个二维点间距离
 * @param[in] pt1
 * @param[in] pt2
 * @return double
 */
inline double pointPointDistance(const cv::Point2f& pt1, const cv::Point2f& pt2) noexcept {
    return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

/**
 * @brief 两个三维点间距离
 * @param[in] pt1
 * @param[in] pt2
 * @return double
 */
inline double pointPointDistance(const cv::Point3f& pt1, const cv::Point3f& pt2) noexcept {
    return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y) +
                     (pt1.z - pt2.z) * (pt1.z - pt2.z));
}

/**
 * @brief 点到直线间距离
 * @param[in] ptP
 * @param[in] ptL1
 * @param[in] ptL2
 * @return double
 */
inline double pointLineDistance(const cv::Point2f& ptP, const cv::Point2f& ptL1,
                                const cv::Point2f& ptL2) noexcept {
    double A = ptL2.y - ptL1.y;
    double B = ptL1.x - ptL2.x;
    double C = ptL2.x * ptL1.y - ptL2.y * ptL1.x;
    double distance = fabs(A * ptP.x + B * ptP.y + C) / sqrt(A * A + B * B);
    return distance;
}

/**
 * @brief 点到直线距离
 * @param[in] pt
 * @param[in] line
 * @return double
 */
inline double pointLineDistance(const cv::Point2f& pt, const cv::Vec4f& line) {
    cv::Vec2f line_dir(line[0], line[1]);
    cv::Point2f line_pt(line[2], line[3]);
    return cv::norm((pt - line_pt).cross(line_dir));
}

/**
 * @brief 求解一元二次方程，返回解的数值对，第一项小于第二项
 * @param[in] a
 * @param[in] b
 * @param[in] c
 * @return std::pair<double, double>
 */
inline std::pair<double, double> solveQuadraticEquation(double a, double b, double c) {
    std::pair<double, double> result((-b - sqrt((double)(b * b - 4 * a * c))) / (2 * a),
                                     (-b + sqrt((double)(b * b - 4 * a * c))) / (2 * a));
    return result;
}

/**
 * @brief 角度转弧度
 * @param[in] angle
 * @return double
 */
inline double angle2Radian(double angle) noexcept { return angle * CV_PI / 180; }

/**
 * @brief 弧度转角度
 * @param[in] radian
 * @return double
 */
inline double radian2Angle(double radian) noexcept { return radian * 180 / CV_PI; }

/**
 * @brief 判断某个值是否在一个范围内。
 * @tparam T
 * @param[in] val           判断的值
 * @param[in] lower         下限
 * @param[in] upper         上限
 * @return true
 * @return false
 */
template <typename T>
constexpr inline bool inRange(T val, T lower, T upper) {
    if (lower > upper) {
        std::swap(lower, upper);
    }
    return val >= lower && val <= upper;
}

}  // namespace power_rune
