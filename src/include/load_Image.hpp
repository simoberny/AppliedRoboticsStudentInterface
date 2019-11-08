
#pragma once
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

void loadImage(cv::Mat& img_out, const std::string& config_folder);
