#pragma once

#include <ceres/ceres.h>

#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <random>
#include <shared_mutex>
#include <thread>

#include "Param.h"
#include "Utility.h"

namespace power_rune {

class Calculator {
   public:
    Calculator();
    ~Calculator();
    bool calculate(const Frame &frame, std::vector<cv::Point2f> &cameraPoints);
    inline cv::Point3f getPredictRobot() { return m_predictRobot; }
    inline cv::Point2f getPredictPixel() { return m_predictPixel; }
    inline std::pair<double, double> getPredictPitchYaw() {
        return std::make_pair(m_predictPitch, m_predictYaw);
    }

   private:
    void preprocess(const Frame &frame, std::vector<cv::Point2f> &cameraPoints);
    bool matrixCal();
    void setFirstDetect();
    void angleCal();
    void directionCal();
    bool predict();
    void fit();
    bool fitOnce();
    std::vector<cv::Point2f> m_cameraPoints;
    std::vector<cv::Point3f> m_worldPoints;
    Direction m_direction;                              // 旋转方向
    Convexity m_convexity;                              // 拟合数据凹凸性
    int m_totalShift;                                   // 总体的装甲板切换数
    double m_bulletSpeed;                               // 子弹速度
    std::chrono::steady_clock::time_point m_frameTime;  // 当前帧的时间戳
    std::chrono::steady_clock::time_point m_startTime;  // 开始的时间戳
    bool m_firstDetect;     // 第一次检测的标志位，第一次检测有效之后置为 true
    cv::Mat m_matW2C;       // 世界坐标系转相机坐标系的 4x4 变换矩阵
    cv::Mat m_matC2G;       // 相机坐标系转云台坐标系的 4x4 变换矩阵
    cv::Mat m_matG2R;       // 云台坐标系转机器人坐标系的 4x4 变换矩阵
    cv::Mat m_matW2R;       // 世界坐标系转机器人坐标系的 4x4 变换矩阵
    cv::Mat m_rMatW2R;      // 世界坐标系转机器人坐标系的 3x3 旋转矩阵
    cv::Mat m_rMatW2RBase;  // 第一次检测有效的世界坐标系转机器人坐标系的 3x3 旋转矩阵
    double m_angleRel;   // 这一帧相对于第一帧的旋转角度（去除了装甲板切换的影响）
    double m_angleLast;  // 上一帧相对于第一帧的旋转角度（不考虑装甲板切换
    double m_distance2Target;
    std::vector<std::pair<double, double>> m_fitData;  // 拟合数据
    std::vector<double> m_directionData;               // 计算旋转方向的数据
    std::array<double, 5> m_params;                    // 拟合参数
    cv::Point3f m_armorRobot;                          // 装甲板中心的机器人坐标
    cv::Point3f m_centerRobot;                         // 中心 R 的机器人坐标
    cv::Point3f m_predictRobot;                        // 预测击打的机器人坐标
    cv::Point2f m_predictPixel;                        // 预测击打的像素坐标
    double m_receiveRoll;                              // 当前帧的roll
    double m_receivePitch;                             // 当前帧的pitch
    double m_receiveYaw;                               // 当前帧的yaw
    double m_predictPitch;
    double m_predictYaw;
    int m_directionThresh;
    std::thread m_fitThread;
    std::shared_mutex m_mutex;
};

/**
 * @brief 惩罚项，让拟合的参数更加贴近预设的参数
 */
class CostFunctor1 : public ceres::SizedCostFunction<1, 5> {
   public:
    CostFunctor1(double truth_, int id_) : truth(truth_), id(id_) {}
    virtual ~CostFunctor1(){};
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        double pre = parameters[0][id];
        residuals[0] = pre - truth;
        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                for (int i = 0; i < 5; ++i) {
                    if (i == id) {
                        jacobians[0][i] = 1;
                    } else {
                        jacobians[0][i] = 0;
                    }
                }
            }
        }
        return true;
    }
    double truth;
    int id;
};

/**
 * @brief 拟合项
 */
class CostFunctor2 : public ceres::SizedCostFunction<1, 5> {
   public:
    CostFunctor2(double t_, double y_) : t(t_), y(y_) {}
    virtual ~CostFunctor2(){};
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        double a = parameters[0][0];
        double w = parameters[0][1];
        double t0 = parameters[0][2];
        double b = parameters[0][3];
        double c = parameters[0][4];
        double cs = cos(w * (t + t0));
        double sn = sin(w * (t + t0));
        residuals[0] = -a * cs + b * t + c - y;
        if (jacobians != NULL) {
            if (jacobians[0] != NULL) {
                jacobians[0][0] = -cs;
                jacobians[0][1] = a * (t + t0) * sn;
                jacobians[0][2] = a * w * sn;
                jacobians[0][3] = t;
                jacobians[0][4] = 1;
            }
        }
        return true;
    }
    double t, y;
};

cv::Mat world2Camera(const std::vector<cv::Point3f> &worldPoints,
                     const std::vector<cv::Point2f> &cameraPoints, const cv::Mat &intrinsicMatrix,
                     const cv::Mat &distCoeffs);
cv::Mat camera2Gimbal(const std::array<double, 3> &rVec, const std::array<double, 3> &tVec);
cv::Mat gimbal2Robot(double pitch, double yaw, double roll);
cv::Point2f getPixelFromCamera(const cv::Mat &intrinsicMatrix, const cv::Mat &cameraPoint);
cv::Point2f getPixelFromRobot(const cv::Point3f &robot, const cv::Mat &w2c, const cv::Mat &w2r);
Convexity getConvexity(const std::vector<std::pair<double, double>> &data);
std::array<double, 5> ransacFitting(const std::vector<std::pair<double, double>> &data, Convexity convexity);
std::array<double, 5> leastSquareEstimate(const std::vector<std::pair<double, double>> &data,
                                          const std::array<double, 5> &params, Convexity convexity);
std::pair<double, double> getPitchYawFromRobotCoor(const cv::Point3f &target, double bulletSpeed);

/**
 * @brief 得到小符旋转角
 * @param[in] distance      到装甲板中心的距离
 * @param[in] bulletSpeed   弹速
 * @param[in] PMSpeed       小符旋转速度
 * @param[in] compansate    时间补偿
 * @return double
 */
inline double getRotationAngleSmall(double distance, double bulletSpeed, double rotationSpeed,
                                    double compansate) noexcept {
    return rotationSpeed * (distance / bulletSpeed + compansate / 1e3);
}

/**
 * @brief 得到大符角度，注意这里是利用参数计算出来的，相对于第一次识别的角度
 * @param[in] time          时间
 * @param[in] params        参数
 * @return double
 */
inline double getAngleBig(double time, const std::array<double, 5> &params) noexcept {
    return -params[0] * std::cos(params[1] * (time + params[2])) + params[3] * time + params[4];
}

/**
 * @brief 得到大符旋转角度，这里是相对于最后一次识别，预测的旋转角
 * @param[in] distance      到装甲板中心的距离
 * @param[in] bulletSpeed   弹速
 * @param[in] params        参数
 * @param[in] compansate    补偿
 * @param[in] frameTime     最后一次识别的时间戳
 * @return double
 */
inline double getRotationAngleBig(double distance, double bulletSpeed, const std::array<double, 5> &params,
                                  double compansate, int64_t frameTime) noexcept {
    double predictTime{distance / bulletSpeed + (frameTime + compansate) * 1e-3};
    double currentTime{frameTime * 1e-3};
    return getAngleBig(predictTime, params) - getAngleBig(currentTime, params);
}

}  // namespace power_rune
