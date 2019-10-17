#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include "include/process_arena.hpp"

// x pc arena: us,ps robotics robtics
//ar_lanch
//altro t: source environment, lancia pipeline

namespace student {

 int image_index = 0;

 void loadImage(cv::Mat& img_out, const std::string& config_folder){  
   throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );
 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
	std::cout << "Saving..." << std::endl;
	char c;
	cv::imshow(topic, img_in);
	c = cv::waitKey(30);
	
	//std::cin >> c;
	if(c=='s'){
		
	
		cv::imwrite(config_folder + "/img_"+std::to_string(student::image_index)+".jpg", img_in);
		std::cout << "Saved!" << std::endl;
		student::image_index++;
	}
  }

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){

	//cv::solvePnP(object_points, img_in, camera_matrix, rvec, tvec);

	//std::cout << "Rotation: " << rvec << " -- Trasla: " << tvec << std::endl;

  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
	std::cout << "undistortion procedure" << std::endl;
	cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);

	//cv::imshow("image undistorted",  img_out);
	//cv::waitKey(30);
  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                          const cv::Mat& tvec, 
                          const std::vector<cv::Point3f>& object_points_plane, 
                          const std::vector<cv::Point2f>& dest_image_points_plane, 
                          cv::Mat& plane_transf, const std::string& config_folder){
    
    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
  }


 void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
              const std::string& config_folder){
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
    //cv::imshow("image unwrapper", img_out);
    //cv::waitKey(20);
  }


  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    
    const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
    if(!res1) std::cout << "processObstacles return false" << std::endl;
    const bool res2 = processGate(hsv_img, scale, gate);
    if(!res2) std::cout << "processGate return false" << std::endl;
    const bool res3 = processVictims(hsv_img, scale, victim_list);
    if(!res3) std::cout << "processVictims return false" << std::endl;

    return res1 && res2 && res3;
  }

/*
  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
	  // vedi libreria find_robo.cpp
  }
  */

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );     
  }
}

