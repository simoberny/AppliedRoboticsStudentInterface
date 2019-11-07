#pragma once

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

const double MIN_AREA_SIZE = 100;

int get_victim_number(cv::Rect boundRect, cv::Mat img, const std::string& config_folder);

