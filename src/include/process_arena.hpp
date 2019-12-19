#pragma once

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

bool processObstacles(const cv::Mat& img_in, cv::Mat &showImage, const double scale, std::vector<Polygon>& obstacle_list, int radius);
bool processGate(const cv::Mat& img_in, cv::Mat &showImage, const double scale, Polygon& gate);
bool processVictims(const cv::Mat& img_in, cv::Mat &showImage, const double scale, std::vector<std::pair<int,Polygon>>& victim_list, const std::string& config_folder);


