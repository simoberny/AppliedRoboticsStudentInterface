#include "include/process_arena.hpp"
#include "include/find_victim.hpp"

//TODO: x ottimizzare la conversion in hsv potrebbe essere fatta 1 solo vaolta!

bool processObstacles(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list){

    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
    
    cv::Mat red_mask_low, red_mask_high, red_mask;
    cv::inRange(hsv_img, cv::Scalar(0, 10, 10), cv::Scalar(15, 255, 255), red_mask_low);
    cv::inRange(hsv_img, cv::Scalar(175, 10, 10), cv::Scalar(179, 255, 255), red_mask_high);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); 

    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;

    
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    
    std::cout << "N. contours: " << contours.size() << std::endl;
    for (int i=0; i<contours.size(); ++i)
    {
      std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
      approxPolyDP(contours[i], approx_curve, 3, true);

      Polygon scaled_contour;
      for (const auto& pt: approx_curve) {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }
      obstacle_list.push_back(scaled_contour);
      contours_approx.push_back(approx_curve);
      drawContours(img_in, contours_approx, -1, cv::Scalar(255,0,0), 3, cv::LINE_AA);
      std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
    }
    //std::cout << std::endl;
    //cv::imshow("Original", hsv_img);
    //cv::waitKey(20);

    return true;
  }

  bool processGate(const cv::Mat& img_in, const double scale, Polygon& gate){

    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
    
    // Find purple regions
    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 50), cv::Scalar(75, 255, 255), green_mask);
    
    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    //cv::Mat contours_img;

    // Process purple mask
    //contours_img = hsv_img.clone();
    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);
    // std::cout << "N. contours: " << contours.size() << std::endl;

    
    bool res = false;

    for( auto& contour : contours){
      const double area = cv::contourArea(contour);
      std::cout << "AREA " << area << std::endl;
      std::cout << "SIZE: " << contours.size() << std::endl;
      if (area > 500){
        approxPolyDP(contour, approx_curve, 8, true);

        contours_approx = {approx_curve};
        drawContours(img_in, contours_approx, -1, cv::Scalar(0,0,255), 3, cv::LINE_AA);


        for (const auto& pt: approx_curve) {
          gate.emplace_back(pt.x/scale, pt.y/scale);
        }
        res = true;
        break;
      }      
    }


    // cv::imshow("Original", contours_img);
    // cv::waitKey(1);
    
    return res;
  }

  bool processVictims(const cv::Mat& img_in, const double scale, std::vector<std::pair<int,Polygon>>& victim_list, const std::string& config_folder){
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
    
    // Find green regions
    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 50), cv::Scalar(75, 255, 255), green_mask);

    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;

    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    //std::cout << "N. contours: " << contours.size() << std::endl;
    for (int i=0; i<contours.size(); ++i)
    {
      //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
      approxPolyDP(contours[i], approx_curve, 1, true);

      if(approx_curve.size() > 6){
        Polygon scaled_contour;
        for (const auto& pt: approx_curve) {
            scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
        }

        int victim_n = get_victim_number(boundingRect(cv::Mat(approx_curve)), img_in, config_folder);

        std::cout << "N: " << victim_n << std::endl;

        victim_list.push_back({victim_n, scaled_contour});
        contours_approx = {approx_curve};

        drawContours(img_in, contours_approx, -1, cv::Scalar(255,100,180), 3, cv::LINE_AA);
        std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
      }
    }
    
    return true;
  }