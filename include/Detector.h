#pragma once

#include <opencv2/opencv.hpp>

#include "Param.h"
#include "Utility.h"

namespace power_rune {

/**
 * @brief 灯条
 */
struct Lightline {
    Lightline() = default;
    Lightline(const std::vector<cv::Point>& contour, const cv::Rect2f& globalRoi,
              const cv::Rect2f& localroi = cv::Rect2f(0, 0, Param::IMAGE_WIDTH, Param::IMAGE_HEIGHT));
    std::vector<cv::Point> m_contour;  // 轮廓点集
    double m_contourArea;              // 轮廓面积
    double m_area;                     // 外接旋转矩形面积
    cv::RotatedRect m_rotatedRect;     // 外接旋转矩形
    cv::Point2f m_tl;                  // 左上角点
    cv::Point2f m_tr;                  // 右上角点
    cv::Point2f m_bl;                  // 左下角点
    cv::Point2f m_br;                  // 右下角点
    cv::Point2f m_center;              // 中心点
    double m_length;                   // 长度
    double m_width;                    // 宽度
    double m_x;                        // 中心点 x 坐标
    double m_y;                        // 中心点 y 坐标
    double m_angle;                    // 旋转矩形角度
    double m_aspectRatio;              // 旋转矩形长宽比
};

/**
 * @brief 装甲板
 */
struct Armor {
    Armor() = default;
    void set(const Lightline& l1, const Lightline& l2);
    inline std::vector<cv::Point2f> getCornerPoints() { return {m_tlIn, m_trIn, m_blOut, m_brOut}; }
    void setCornerPoints(const std::vector<cv::Point2f>& points);
    Lightline m_inside;    // 内部灯条
    Lightline m_outside;   // 外部灯条
    cv::Point2f m_tlIn;    // 内部灯条左上角点
    cv::Point2f m_trIn;    // 内部灯条右上角点
    cv::Point2f m_blIn;    // 内部灯条左下角点
    cv::Point2f m_brIn;    // 内部灯条右下角点
    cv::Point2f m_tlOut;   // 外部灯条左上角点
    cv::Point2f m_trOut;   // 外部灯条右上角点
    cv::Point2f m_blOut;   // 外部灯条左下角点
    cv::Point2f m_brOut;   // 外部灯条右下角点
    cv::Point2f m_center;  // 装甲板中心点
    double m_x;            // 装甲板中心 x 坐标
    double m_y;            // 装甲板中心 y 坐标
};

/**
 * @brief 中心 R
 */
struct CenterR {
    CenterR() = default;
    void set(const Lightline& contour);
    Lightline m_lightline;    // 中心 R 灯条
    cv::Point2f m_center;     // 中心 R 点
    cv::Rect m_boundingRect;  // 中心 R 最小正矩形
    double m_x;               // 中心 R x 坐标
    double m_y;               // 中心 R y 坐标
};

/**
 * @brief 箭头
 */
struct Arrow {
    Arrow() = default;
    void set(const std::vector<Lightline>& points, const cv::Point2f& roi);
    std::vector<cv::Point> m_contour;  // 轮廓点集
    cv::RotatedRect m_rotatedRect;     // 外接旋转矩形
    double m_length;                   // 长度
    double m_width;                    // 宽度
    cv::Point2f m_center;              // 中心点
    double m_angle;                    // 角度
    double m_aspectRatio;              // 长宽比
    double m_area;                     // 面积
    double m_fillRatio;                // 填充比例
};

void findArrowLightlines(const cv::Mat& binary, std::vector<Lightline>& lightlines, const cv::Rect2f& roi);
bool findArrow(Arrow& arrow, const std::vector<Lightline>& lightlines, const cv::Rect2f& roi);
bool sameArrow(const Lightline& l1, const Lightline& l2);
bool sameArmor(const Lightline& l1, const Lightline& l2);
bool findArmor(Armor& armor, const std::vector<Lightline>& frames, const Arrow& arrow);
bool findArmorLightlines(const cv::Mat& binary, std::vector<Lightline>& frames, const cv::Rect2f& globalRoi,
                         const cv::Rect2f& localRoi);
bool findCenterLightlines(const cv::Mat& binary, std::vector<Lightline>& lightlines,
                          const cv::Rect2f& globalRoi, const cv::Rect2f& localRoi);
bool findCenterR(CenterR& center, const std::vector<Lightline>& lightlines, const Arrow& arrowPtr,
                 const Armor& armor);
void resetRoi(cv::Rect2f& rect, const cv::Mat& mat);
void resetRoi(cv::Rect2f& rect, int rows, int cols);
double calAngleBetweenLightlines(const Lightline& l1, const Lightline& l2);
void resetRoi(cv::Rect2f& rect, const cv::Mat& mat);
void resetRoi(cv::Rect2f& rect, const cv::Rect2f& lastRoi);
void resetRoi(cv::Rect2f& rect, int rows, int cols);
bool inRect(const cv::Point2f& point, const cv::Rect2f& rect);

/**
 * @brief 检测类，负责对图像的处理和目标的检测，得到所有特征点的像素坐标，以及点亮的装甲板数目。
 */
class Detector {
   public:
    Detector();
    bool detect(const Frame& frame);
    void drawTargetPoint(const cv::Point2f& point);
    inline void visualize() { cv::imshow("visualized", m_imageShow); }

    /**
     * @brief
     * 得到像素坐标系特征点，分别为装甲板内灯条的左上，右上，外灯条的中上，左下，右下，中心R。
     * @return std::vector<cv::Point2f>
     */
    inline std::vector<cv::Point2f> getCameraPoints() {
        return {m_armor.m_tlIn,  m_armor.m_trIn,  (m_armor.m_tlOut + m_armor.m_trOut) * 0.5,
                m_armor.m_blOut, m_armor.m_brOut, m_centerR.m_center};
    }

   private:
    cv::Mat m_imageRaw;      // 原图
    cv::Mat m_imageArrow;    // 检测箭头用的二值化图片
    cv::Mat m_imageArmor;    // 检测装甲板边框用的二值化图片
    cv::Mat m_imageCenter;   // 检测中心 R 用的二值化图片
    cv::Mat m_imageShow;     // 可视化图片
    cv::Mat m_localMask;     // 局部 roi 的掩码
    cv::Rect2f m_globalRoi;  // 全局 roi ，用来圈定识别的范围，加快处理速度
    cv::Rect2f m_armorRoi;   // 装甲板 roi
    cv::Rect2f m_centerRoi;  // 中心 R roi
    Arrow m_arrow;           // 箭头
    Armor m_armor;           // 装甲板
    CenterR m_centerR;       // 中心 R
    std::chrono::steady_clock::time_point m_startTime;  // 检测开始的时间戳
    std::chrono::steady_clock::time_point m_frameTime;  // 当前帧的时间戳
    int m_lightArmorNum;                                // 点亮的装甲板数目
    Status m_status;  // 检测标志，包括成功、箭头检测失败、装甲板检测失败、中心 R 检测失败
    void preprocess(const Frame& frame);
    bool detectArrow();
    void setLocalRoi();
    bool detectArmor();
    bool detectCenterR();
    void setArmor();
    void setGlobalRoi();
    void draw(const Lightline& lightline, const cv::Scalar& color, const int thickness = 1,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, Param::IMAGE_WIDTH, Param::IMAGE_HEIGHT));
    void draw(const cv::RotatedRect& rotatedRect, const cv::Scalar& color, const int thickness = 1,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, Param::IMAGE_WIDTH, Param::IMAGE_HEIGHT));
    void draw(const cv::Rect2f& rect, const cv::Scalar& color, const int thickness = 1,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, Param::IMAGE_WIDTH, Param::IMAGE_HEIGHT));
    void draw(const std::vector<cv::Point2f>& points, const cv::Scalar& color, const int thickness = 1,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, Param::IMAGE_WIDTH, Param::IMAGE_HEIGHT));
    void draw(const cv::Point2f* points, const size_t size, const cv::Scalar& color, const int thickness = 1,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, Param::IMAGE_WIDTH, Param::IMAGE_HEIGHT));
};

}  // namespace power_rune
