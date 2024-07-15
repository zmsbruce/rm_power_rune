#include "Param.h"

namespace power_rune {

Param::Param(const std::string& filename) { load(filename); }

void Param::load(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    // color
    std::string colorStr;
    fs["color"] >> colorStr;
    if (std::transform(colorStr.begin(), colorStr.end(), colorStr.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        colorStr == "red") {
        COLOR = Color::RED;
    } else if (colorStr == "blue") {
        COLOR = Color::BLUE;
    } else {
        throw std::runtime_error("unknown color " + colorStr);
    }
    DRAW_COLOR = COLOR == Color::BLUE ? RED : BLUE;
    // fps
    fs["fps"] >> FPS;
    // image width and height
    auto fsImage = fs["image"];
    fsImage["width"] >> IMAGE_WIDTH;
    fsImage["height"] >> IMAGE_HEIGHT;
    // brightness threshold
    auto fsDetect = fs["detect"];
    auto fsBrightness = fsDetect["brightness_threshold"][colorStr];
    fsBrightness["arrow"] >> ARROW_BRIGHTNESS_THRESHOLD;
    fsBrightness["armor"] >> ARMOR_BRIGHTNESS_THRESHOLD;
    // local roi
    auto fsLocalRoi = fsDetect["local_roi"];
    fsLocalRoi["distance_ratio"] >> LOCAL_ROI_DISTANCE_RATIO;
    fsLocalRoi["width"] >> LOCAL_ROI_WIDTH;
    // vertical distance thresh from armor to centerR
    fsDetect["armor_center_vertical_distance_threshold"] >> ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD;
    // global roi length ratio
    fsDetect["global_roi_length_ratio"] >> GLOBAL_ROI_LENGTH_RATIO;
    // arrow
    auto fsArrow = fsDetect["arrow"];
    fsArrow["lightline"]["area"]["min"] >> MIN_ARROW_LIGHTLINE_AREA;
    fsArrow["lightline"]["area"]["max"] >> MAX_ARROW_LIGHTLINE_AREA;
    fsArrow["lightline"]["aspect_ratio_max"] >> MAX_ARROW_LIGHTLINE_ASPECT_RATIO;
    fsArrow["lightline"]["num"]["min"] >> MIN_ARROW_LIGHTLINE_NUM;
    fsArrow["lightline"]["num"]["max"] >> MAX_ARROW_LIGHTLINE_NUM;
    fsArrow["same_area_ratio_max"] >> MAX_SAME_ARROW_AREA_RATIO;
    fsArrow["aspect_ratio"]["min"] >> MIN_ARROW_ASPECT_RATIO;
    fsArrow["aspect_ratio"]["max"] >> MAX_ARROW_ASPECT_RATIO;
    fsArrow["area_max"] >> MAX_ARROW_AREA;
    // armor
    auto fsArmor = fsDetect["armor"];
    fsArmor["lightline"]["area"]["min"] >> MIN_ARMOR_LIGHTLINE_AREA;
    fsArmor["lightline"]["area"]["max"] >> MAX_ARMOR_LIGHTLINE_AREA;
    fsArmor["lightline"]["contour_area"]["min"] >> MIN_ARMOR_LIGHTLINE_CONTOUR_AREA;
    fsArmor["lightline"]["contour_area"]["max"] >> MAX_ARMOR_LIGHTLINE_CONTOUR_AREA;
    fsArmor["lightline"]["aspect_ratio"]["min"] >> MIN_ARMOR_LIGHTLINE_ASPECT_RATIO;
    fsArmor["lightline"]["aspect_ratio"]["max"] >> MAX_ARMOR_LIGHTLINE_ASPECT_RATIO;
    fsArmor["same"]["area_ratio_max"] >> MAX_SAME_ARMOR_AREA_RATIO;
    fsArmor["same"]["distance"]["min"] >> MIN_SAME_ARMOR_DISTANCE;
    fsArmor["same"]["distance"]["max"] >> MAX_SAME_ARMOR_DISTANCE;
    // centerR
    auto fsCenterR = fsDetect["centerR"];
    fsCenterR["area"]["min"] >> MIN_CENTER_AREA;
    fsCenterR["area"]["max"] >> MAX_CENTER_AREA;
    fsCenterR["aspect_ratio_max"] >> MAX_CENTER_ASPECT_RATIO;
    // mode
    std::string modeStr;
    fs["mode"] >> modeStr;
    if (std::transform(modeStr.begin(), modeStr.end(), modeStr.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        modeStr == "small") {
        MODE = Mode::SMALL;
    } else if (modeStr == "big") {
        MODE = Mode::BIG;
    } else {
        throw std::runtime_error("unknown mode " + modeStr);
    }
    // bullet_speed
    auto fsCal = fs["calculate"];
    fsCal["bullet_speed"]["min"] >> MIN_BULLET_SPEED;
    fsCal["bullet_speed"]["default"] >> DEFAULT_BULLET_SPEED;
    // tvec_c2g
    auto fsc2g = fsCal["tvec_c2g"];
    if (fsc2g.type() == cv::FileNode::SEQ) {
        int index = 0;
        for (auto it = fsc2g.begin(); it != fsc2g.end(); it++) {
            CAMERA_TO_GIMBAL_TRANSLATION_VECTOR.at(index) = static_cast<double>(*it);
            index++;
        }
    }
    // compansate
    auto fsCompansate = fsCal["compansate"];
    fsCompansate["time"] >> COMPANSATE_TIME;
    fsCompansate["pitch"] >> COMPANSATE_PITCH;
    fsCompansate["yaw"] >> COMPANSATE_YAW;
    // camera parameters
    fsCal["intrinsic_matrix"] >> INTRINSIC_MATRIX;
    fsCal["distortion"] >> DIST_COEFFS;
    // armor parameters
    fsArmor = fsCal["armor"];
    fsArmor["outside"]["width"] >> ARMOR_OUTSIDE_WIDTH;
    fsArmor["outside"]["height"] >> ARMOR_OUTSIDE_HEIGHT;
    fsArmor["outside"]["y"] >> ARMOR_OUTSIDE_Y;
    fsArmor["inside"]["width"] >> ARMOR_INSIDE_WIDTH;
    fsArmor["inside"]["y"] >> ARMOR_INSIDE_Y;
    // fit data size
    fsCal["fit_data_size"]["min"] >> MIN_FIT_DATA_SIZE;
    fsCal["fit_data_size"]["max"] >> MAX_FIT_DATA_SIZE;
}

}  // namespace power_rune