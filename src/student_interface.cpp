#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>
#include <tuple>

#include "include/dubins.hpp"
#include "include/process_arena.hpp"
#include "include/Voronoi.hpp"
#include "include/clipper.hpp"

#include "include/find_collision.hpp"

#include <fstream>
using namespace std;

namespace student {
    //Robot radius (to enlarge obstacles)
    int robot_r = 65;
    //enlarged border raius
    int border_radius = 0;

    int image_index = 0;

    void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder) {
        std::cout << "Saving..." << std::endl;
        char c;
        cv::imshow(topic, img_in);
        c = cv::waitKey(30);

        if (c == 's') {
            cv::imwrite(config_folder + "/img_" + std::to_string(student::image_index) + ".jpg", img_in);
            std::cout << "Saved!" << std::endl;
            student::image_index++;
        }
    }

    //-------------------------------------------------------------------------
    //          EXTRINSIC CALIB IMPLEMENTATION
    //-------------------------------------------------------------------------

    // Defintion of the function pickNPoints and the callback mouseCallback.
    // The function pickNPoints is used to display a window with a background
    // image, and to prompt the user to select n points on this image.
    static cv::Mat bg_img;
    static std::vector<cv::Point2f> result;
    static std::string name;
    static std::atomic<bool> done;
    static int n;
    static double show_scale = 1.0;

    void mouseCallback(int event, int x, int y, int, void *p) {
        if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;

        result.emplace_back(x * show_scale, y * show_scale);
        cv::circle(bg_img, cv::Point(x, y), 20 / show_scale, cv::Scalar(0, 0, 255), -1);
        cv::imshow(name.c_str(), bg_img);

        if (result.size() >= n) {
            usleep(500 * 1000);
            done.store(true);
        }
    }

    std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat &img) {
        result.clear();
        cv::Size small_size(img.cols / show_scale, img.rows / show_scale);
        cv::resize(img, bg_img, small_size);
        //bg_img = img.clone();
        name = "Pick " + std::to_string(n0) + " points";
        cv::imshow(name.c_str(), bg_img);
        cv::namedWindow(name.c_str());
        n = n0;

        done.store(false);

        cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
        while (!done.load()) {
            cv::waitKey(500);
        }

        cv::destroyWindow(name.c_str());
        return result;
    }


    bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix,
                        cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder) {
        std::string file_path = config_folder + "/extrinsicCalib.csv";

        std::vector<cv::Point2f> image_points;

        if (!std::experimental::filesystem::exists(file_path)) {

            std::experimental::filesystem::create_directories(config_folder);

            image_points = pickNPoints(4, img_in);

            std::ofstream output(file_path);
            if (!output.is_open()) {
                throw std::runtime_error("Cannot write file: " + file_path);
            }
            for (const auto pt: image_points) {
                output << pt.x << " " << pt.y << std::endl;
            }
            output.close();
        } else {
            // LOAD POINT FROM FILE
            std::ifstream input(file_path);
            if (!input.is_open()) {
                throw std::runtime_error("Cannot read file: " + file_path);
            }
            while (!input.eof()) {
                double x, y;
                if (!(input >> x >> y)) {
                    if (input.eof()) break;
                    else {
                        throw std::runtime_error("Malformed file: " + file_path);
                    }
                }
                image_points.emplace_back(x, y);
            }
            input.close();
        }

        cv::Mat dist_coeffs;
        dist_coeffs = (cv::Mat1d(1, 4) << 0, 0, 0, 0, 0);
        bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

        if (!ok)
            std::cerr << "FAILED SOLVE_PNP" << std::endl;

        return ok;
    }

    void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out,
                        const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const std::string &config_folder) {
        cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);

        //cv::imshow("image undistorted",  img_out);
        //cv::waitKey(30);
    }

    void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec,
                            const cv::Mat &tvec,
                            const std::vector<cv::Point3f> &object_points_plane,
                            const std::vector<cv::Point2f> &dest_image_points_plane,
                            cv::Mat &plane_transf, const std::string &config_folder) {
        cv::Mat image_points;

        // project points
        cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

        plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
    }

    void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf,
                const std::string &config_folder) {
        cv::warpPerspective(img_in, img_out, transf, img_in.size());
    }

    // Function to get the centroid point of a Polygon
    std::pair<double, double> calcCentroid(const Polygon& p) {
        double max_x = 0, max_y = 0, min_x = 1000, min_y = 1000;

        for (auto a : p) {
            if (a.x > max_x) max_x = a.x;
            if (a.x < min_x) min_x = a.x;
            if (a.y > max_y) max_y = a.y;
            if (a.y < min_y) min_y = a.y;
        }

        double cx = max_x - ((max_x - min_x) / 2);
        double cy = max_y - ((max_y - min_y) / 2);

        return make_pair(cx, cy);
    }

    // Comparator function to sort a vector containing tuple
    bool comparePair(tuple<double, double, double> i1, tuple<double, double, double> i2) {
        return (get<2>(i1) < get<2>(i2));
    }

    bool compareLength(pair<int, double> i1, pair<int, double> i2) {
        return (i1.second < i2.second);
    }

    Polygon resizeBorders(const Polygon &borders,  double resize){
        Polygon re_border;
        for(auto &pt: borders){
            double b_x = pt.x;
            double b_y = pt.y;

            if(b_x < 0.50){
                b_x += resize;
            }else{
                b_x -= resize;
            }

            if(b_y < 0.50){
                b_y += resize;
            }else{
                b_y -= resize;
            }

            re_border.push_back(Point(b_x, b_y));
        }

        return re_border;
    }

    std::vector<Polygon> enlargeObstacle(const std::vector<Polygon> ob, int radius){
        std::vector<Polygon> enlarged;

        for(int i = 0; i < ob.size(); i++){
            ClipperLib::Path srcPoly;
            ClipperLib::Paths newPoly;

            Polygon enlargePoly;

            const double INT_ROUND = 1000;

            for(size_t a = 0; a < ob[i].size(); ++a){
                int x = ob[i][a].x * INT_ROUND;
                int y = ob[i][a].y * INT_ROUND;

                srcPoly << ClipperLib::IntPoint(x, y);
            }

            ClipperLib::ClipperOffset co;

            co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);

            co.Execute(newPoly, radius);

            for(const ClipperLib::Path &path: newPoly){
                for(const ClipperLib::IntPoint &pt: path){
                    double x = pt.X / INT_ROUND;
                    double y = pt.Y / INT_ROUND;
                    enlargePoly.emplace_back(x, y);
                }
            }

            enlarged.emplace_back(enlargePoly);
        }

        return enlarged;
    }

    std::vector<Polygon> mergePolygon(const std::vector<Polygon> &obstacle_list, const Polygon &borders){
        std::vector<Polygon> new_list;
        std::vector<Polygon> clipped_list;
        ClipperLib::Paths solution;
        ClipperLib::Clipper c;
        ClipperLib::Paths clip(1);

        for(int i= 0; i < obstacle_list.size(); i++){
            ClipperLib::Paths subj(1);
            for (const auto &pt: obstacle_list[i]) {
                subj[0] << ClipperLib::IntPoint(pt.x*1000, pt.y*1000);
            }

            c.AddPaths(subj, ClipperLib::ptSubject, true);
        }

        c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        for(int i= 0; i < solution.size(); i++){
            Polygon p;
            for (const auto &pt: solution[i]) {
                p.push_back(Point((double) pt.X/1000.0, (double)pt.Y/1000.0));
            }

            new_list.emplace_back(p);
        }

        ClipperLib::Clipper inter;
        ClipperLib::Paths sol2;

        for(int i= 0; i < new_list.size(); i++){
            ClipperLib::Paths subj(1);
            for (const auto &pt: new_list[i]) {
                subj[0] << ClipperLib::IntPoint(pt.x*1000, pt.y*1000);
            }

            inter.AddPaths(subj, ClipperLib::ptSubject, true);
        }

        for(auto &pt: borders){
            clip[0] << ClipperLib::IntPoint(pt.x*1000, pt.y*1000);
        }

        inter.AddPaths(clip, ClipperLib::ptClip, true);
        inter.Execute(ClipperLib::ctIntersection, sol2, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        for(int i= 0; i < sol2.size(); i++){
            Polygon p;
            for (const auto &pt: sol2[i]) {
                p.push_back(Point((double) pt.X/1000.0, (double)pt.Y/1000.0));
            }

            clipped_list.emplace_back(p);
        }


        return clipped_list;
    }

    bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list,
                    std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate,
                    const std::string &config_folder) {
        //aumento luminosità.... penultimo parametro é alfa, ultimo è beta  pixel = pixel*alfa+beta
        img_in.convertTo(img_in, -1, 1.3, 0);
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
                img.at<cv::Vec3b>(i,j)[idx] += 2.3;
            }
        }

        // HSV back to BGR
        cv::cvtColor(img, img_in, cv::COLOR_HSV2BGR);
        cv::Mat showImage = img_in.clone();

        bool arena = true;

        string string;
        ifstream infile;
        infile.open (config_folder + "setup.txt");

        int l = 0;
        while(!infile.eof()) // To get you all the lines.
        {
            getline(infile,string); // Saves the line in STRING.

            if (l == 1){
                if(string.compare("arena")) arena = true;
                if(string.compare("Arena")) arena = true;
                if(string.compare("simulator")) arena = false;
                if(string.compare("Simulator")) arena = false;
            }

            l++;
        }

        infile.close();

        std::cout << "enter in process map" << std::endl;
        const bool res1 = processObstacles(img_in, showImage, scale, obstacle_list, robot_r);
        if (!res1) std::cout << "processObstacles return false" << std::endl;
        const bool res2 = processGate(img_in, showImage, scale, gate, arena);
        if (!res2) std::cout << "processGate return false" << std::endl;
        const bool res3 = processVictims(img_in, showImage, scale, victim_list, config_folder, arena);
        if (!res3) std::cout << "processVictims return false" << std::endl;

        cv::imwrite(config_folder + "/img_obstacle_recognition.jpg",showImage);

        return res1 && res2;
    }

    bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                  const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                  const float y, const float theta, Path &path, const string &config_folder) {
        std::cout << "\n\033[1;36m#Planning path... \033[0m\n" << std::endl;

        //System to now how much time takes the plan
        auto started = std::chrono::high_resolution_clock::now();

        // Take configuration from file
        int program = 2;
        int kmax = 15;

        string string;

        ifstream infile;
        infile.open (config_folder+"setup.txt");
        int l = 0;

        while(!infile.eof()) // To get you all the lines.
        {
            getline(infile,string); // Saves the line in STRING.

            if (l == 3){ program = std::stoi(string); }
            if (l == 5){ kmax = std::stoi(string); }

            l++;
        }

        infile.close();

        //Resize of the arena 
        Polygon obstacle_border = resizeBorders(borders, 0.07);
        Polygon voronoi_border = resizeBorders(borders, 0.06);

        //Enlargement and Merge of the interecating polygon
        std::vector<Polygon> englarge_obstacle = enlargeObstacle(obstacle_list, robot_r);
        std::vector<Polygon> merged_list = mergePolygon(englarge_obstacle, obstacle_border);

        //Enlargement and merge bigger for prunig level 2
        std::vector<Polygon> clean_obstacle = enlargeObstacle(obstacle_list, robot_r + 30);
        std::vector<Polygon> clean_merged_list = mergePolygon(clean_obstacle, obstacle_border);

        //Initialize voronoi diagram
        voronoi_diagram<double> vd;
        Voronoi v;

        //Calculate the voronoi points
        double gate_angle;
        v.calculate(merged_list, voronoi_border, borders, victim_list, gate, x, y, theta, vd, gate_angle);

        //Generate the graph
        std::vector<std::tuple<int, Voronoi::Point, double> > t = v.graph(vd,merged_list, theta, gate_angle, program, clean_merged_list);

        //Draw all the scene
        cv::Mat image = v.draw(merged_list, voronoi_border, victim_list, gate, x, y, theta, vd, t);
        cv::imwrite(config_folder + "/img_voronoi.jpg", image);

        static double scale = 500.0;

        Path final_path;

        //Initial robot position
        double rob_x = x;
        double rob_y = y;
        double rob_theta = theta;

        //Gate centroid = final position
        pair<double, double> gate_centroid = calcCentroid(gate);
        double x_gate = gate_centroid.first;
        double y_gate = gate_centroid.second;

        int k = t.size();

        //Foreach victim to reach
        for (int a = 1; a < k; a++) {
            int id;
            Voronoi::Point p(0,0);
            double xf, yf, angle;
            tie(id, p, angle) = t[a];

            xf = p.a / scale;
            yf = p.b / scale;

            //Get the dubins curve
            Dubins dub;
            dub.setParams(rob_x, rob_y, rob_theta, xf, yf, angle, kmax);

            pair<int, curve> ret = dub.shortest_path();
            curve cur = ret.second;

            Path temp = dub.getPath(cur);
            for (const auto &point : temp.points) {
                final_path.points.push_back(point);
            }

            //Sets the actual robot position
            rob_x = xf;
            rob_y = yf;
            rob_theta = angle;
        }

        auto done = std::chrono::high_resolution_clock::now();

        cout << "\033[1;31mTEMPO DI ESECUZIONE: " << std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count() << "\033[0m\n" << endl;

        //Send the path to the robot
        path = final_path;
    }
}