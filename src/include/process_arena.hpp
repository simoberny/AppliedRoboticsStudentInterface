#pragma once

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list);
bool processGate(const cv::Mat& hsv_img, const double scale, Polygon& gate);
bool processVictims(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list);


