#include "include/find_robot.hpp"

namespace student {
    bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta,
                   const std::string &config_folder) {

        //aumento luminosità.... penultimo parametro é alfa, ultimo è beta  pixel = pixel*alfa+beta
        img_in.convertTo(img_in, -1, 1.5, 0);
        //aumento della staturazione
        // BGR to HSV
        cv::Mat img;
        cv::cvtColor(img_in, img, cv::COLOR_BGR2HSV);
        for (int i=0; i < img.rows ; i++)
        {
            for(int j=0; j < img.cols; j++)
            {
                // You need to check this, but I think index 1 is for saturation, but it might be 0 or 2
                int idx = 1;
                img.at<cv::Vec3b>(i,j)[idx] += 1.8;
            }
        }


        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

        cv::Mat blue_mask;
        //cv::inRange(hsv_img, cv::Scalar(100, 120, 150), cv::Scalar(135, 255, 255), blue_mask);
        cv::inRange(hsv_img, cv::Scalar(100, 100, 50), cv::Scalar(130, 255, 255), blue_mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(blue_mask, contours,
                         cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(img_in, contours, -1, cv::Scalar(40, 190, 40), 4, cv::LINE_AA);

        cv::imwrite(config_folder + "/img_robot.jpg", img_in);

        cv::imshow("robot_mask", blue_mask);
        cv::waitKey(20);

        std::vector<cv::Point> approx_curve;

        for (int i = 0; i < contours.size(); ++i)    //se per caso il blue mask trova + di 1 figura scelgo sono quella che approssimata ha 3 lati!
        {
            // Approximate the i-th contours
            cv::approxPolyDP(contours[i], approx_curve, 10, true);


            // Check the number of edge of the aproximate contour

            if (approx_curve.size() == 3) {
                //std::cout << "triangle (robot) found!:  " << contours[i].size() << std::endl;
                break;
            }
        }

        double area = cv::contourArea(approx_curve);

        if (approx_curve.size() == 3 && area > 400) {
/*
            std::cout << "-----Aprox Contour size: " << approx_curve.size() << std::endl;
            std::cout << "Area: " << area << std::endl;
            std::cout << "punti trovati: 1 " << std::endl;
            std::cout << " x " << approx_curve[0].x << " y " << approx_curve[0].y << std::endl;
            std::cout << " x " << approx_curve[1].x << " y " << approx_curve[1].y << std::endl;
            std::cout << " x " << approx_curve[2].x << " y " << approx_curve[2].y << std::endl;
*/
            for (const auto &pt: approx_curve) {
                triangle.emplace_back(pt.x / scale, pt.y / scale);
            }

            double cx, cy;
            for (auto item: triangle) {
                cx += item.x;
                cy += item.y;
            }
            cx /= triangle.size();
            cy /= triangle.size();

            double dst = 0;
            Point vertex;
            for (auto &item: triangle) {
                double dx = item.x - cx;
                double dy = item.y - cy;
                double curr_d = dx * dx + dy * dy;
                if (curr_d > dst) {
                    dst = curr_d;
                    vertex = item;
                }
            }
            double dx = cx - vertex.x;
            double dy = cy - vertex.y;

            x = cx;
            y = cy;
            theta = std::atan2(dy, dx);

            //drawContours vuole in input un vettore di vettori di posizioni (approx_curve è solo un vettore)
            std::vector<std::vector<cv::Point>> vec_approx_curve;
            vec_approx_curve = {approx_curve};
            cv::drawContours(img_in, vec_approx_curve, -1, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
            //calcolo la posizione del robot (rotazione e posizione del vertice del triangolo)
/*
            std::cout << "-----triangle position (in meter?):  " << std::endl;
            std::cout << " x " << triangle[0].x << " y " << triangle[0].y << std::endl;
            std::cout << " x " << triangle[1].x << " y " << triangle[1].y << std::endl;
            std::cout << " x " << triangle[2].x << " y " << triangle[2].y << std::endl;
*/

            return true;


        } else {
            return false;
        }
    }

}