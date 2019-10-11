#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

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
//rvec vettore di rotazioni camera      tvec vettore 3d della posizione della camera (NOTA nel sistema di riferimento della camera!)

    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );   
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
	std::cout << "undistortion procedure" << std::endl;
	cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);

	//cv::imshow("img_in", img_in);
	//cv::imshow("img_out", img_out);
	//cv::waitKey(20);


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
    cv::imshow("image unwrapper", img_out);
    cv::waitKey(20);
  }


  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat mask_red;	//trova gli ostacoli	
//mat è una shered pointer (è generato dinamicamente, ma ha un contatore per sapere quando non c'è più nessuno che punta ad essa)
//per clonare un'immagine usare img_clone()
    cv::Mat hsv_img;

    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    cv::inRange(hsv_img, cv::Scalar(0, 10, 10), cv::Scalar(15, 255, 255), mask_red);   

    cv::findContours(mask_red, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(img_in, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);

    //cv::imshow("mask", img_in);
    //cv::waitKey(20);    

  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
	  cv::Mat hsv_img;
	  cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

  	cv::Mat blue_mask;    
	  cv::inRange(hsv_img, cv::Scalar(100, 120, 150), cv::Scalar(135, 255, 255), blue_mask);

  	std::vector<std::vector<cv::Point>> contours;    
  	cv::findContours(blue_mask, contours, 
                      cv::RETR_EXTERNAL, 
                      cv::CHAIN_APPROX_SIMPLE);
  	cv::drawContours(img_in, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);
	
	  cv::imshow("robot_mask", img_in);
    	cv::waitKey(20);  

  	std::vector<cv::Point> approx_curve;
    
  	for (int i=0; i<contours.size(); ++i)	//se per caso il blue mask trova + di 1 figura scelgo sono quella che approssimata ha 3 lati!
  	{
  	  // Approximate the i-th contours      
  	  cv::approxPolyDP(contours[i], approx_curve, 30, true);

  	  // Check the number of edge of the aproximate contour
  	  if (approx_curve.size() == 3){
        std::cout << "rectangle (robot) found!:  " << contours[i].size() << std::endl;
        break;
      }
    }
    double area = cv::contourArea(approx_curve);

	  std::cout <<" Aprox Contour size: " << approx_curve.size() << std::endl;
	  std::cout << "Area: " << area << std::endl;

    //bìdrawContours vuole in input un vettore di vettori di posizioni (approx_curve è solo un vettore)
    std::vector<std::vector<cv::Point>> vec_approx_curve;    
    vec_approx_curve = {approx_curve};
	  cv::drawContours(img_in, vec_approx_curve, -1, cv::Scalar(0,0,255), 3, cv::LINE_AA);

	  

	
	
	

  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );     
  }


}

