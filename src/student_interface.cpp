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
/*
//utilizzare per rettangolo che ingloba il poligono e stima dell'area tra poligoni
#include <iostream>

#include "include/boost/geometry.hpp"
#include "include/boost/geometry/geometries/box.hpp"
#include "include/boost/geometry/geometries/point_xy.hpp"
#include "include/boost/geometry/geometries/polygon.hpp"
#include "include/boost/geometry/io/wkt/wkt.hpp"

#include <deque>

#include <boost/foreach.hpp>
 */

#include "include/find_collision.hpp"

//namespace geom = boost::geometry;

// x pc arena: us,ps robotics robtics
//ar_lanch
//altro t: source environment, lancia pipeline

namespace student {

    int image_index = 0;

    void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder) {
        std::cout << "Saving..." << std::endl;
        char c;
        cv::imshow(topic, img_in);
        c = cv::waitKey(30);


        //std::cin >> c;
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
            // SAVE POINT TO FILE
            // std::cout << "IMAGE POINTS: " << std::endl;
            // for (const auto pt: image_points) {
            //   std::cout << pt << std::endl;
            // }
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

        // cv::Mat Rt;
        // cv::Rodrigues(rvec_, Rt);
        // auto R = Rt.t();
        // auto pos = -R * tvec_;

        if (!ok) {
            std::cerr << "FAILED SOLVE_PNP" << std::endl;
        }

        return ok;
    }

    void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out,
                        const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const std::string &config_folder) {
        std::cout << "undistortion procedure" << std::endl;
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
        //cv::imshow("image unwrapper", img_out);
        //cv::waitKey(20);
    }

    // Recursive function to get all the combination with repetition
    void getPermutationRec(int set[], const vector<int> iter, int n, int k, vector<vector<int>> &des) {
        if (k == 0) {
            des.push_back(iter);
            /*for(int i = 0; i < iter.size(); i++){
                std::cout << iter[i];
            }

            std::cout << endl;*/
            return;
        }

        for (int i = 0; i < n; i++) {
            vector<int> temp = iter;
            temp.push_back(set[i]);
            getPermutationRec(set, temp, n, k - 1, des);
        }
    }

    // Wrapper for permutation recursive function
    vector<vector<int>> getPermutation(int set[], const vector<int>& iter, int n, int k) {
        // Global vector to save all the possible victim descent
        vector<vector<int>> descent;
        // Call the recursive version
        getPermutationRec(set, iter, n, k, descent);
        return descent;
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

    bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list,
                    std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate,
                    const std::string &config_folder) {
        //Robot radius
        int robot_r = 5;

        std::cout << "enter in process map" << std::endl;
        const bool res1 = processObstacles(img_in, scale, obstacle_list, robot_r);
        if (!res1) std::cout << "processObstacles return false" << std::endl;
        const bool res2 = processGate(img_in, scale, gate);
        if (!res2) std::cout << "processGate return false" << std::endl;
        const bool res3 = processVictims(img_in, scale, victim_list, config_folder);
        if (!res3) std::cout << "processVictims return false" << std::endl;


        cv::imshow("Original", img_in);
        cv::waitKey(1000);

        return res1 && res2 && res3;
    }

    bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                  const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                  const float y, const float theta, Path &path, const string &config_folder) {
        voronoi_diagram<double> vd;
        Voronoi v;
        v.calculate(obstacle_list, borders, victim_list, gate, x, y, theta, vd);
        std::vector<std::tuple<int, Voronoi::Point, double> > t = v.graph(vd);

        //v.draw(obstacle_list, borders, victim_list, gate, x, y, theta, vd, t);

        std::cout << "Numero punti path: " << t.size() << std::endl;

        Path final_path;

        vector<pair<int, double>> descent_leading;

        //System to now how much time takes the plan
        auto started = std::chrono::high_resolution_clock::now();

        //Initial robot position
        double rob_x = x;
        double rob_y = y;
        double rob_theta = 0;

        //Possible angles
        //vector<double> angles = {0, M_PI / 2, M_PI, -M_PI / 2};

        //Gate centroid = final position
        pair<double, double> gate_centroid = calcCentroid(gate);
        double x_gate = gate_centroid.first;
        double y_gate = gate_centroid.second;

        /*vector<int> te;
        //Generate array of combination elements
        int angle_idx[angles.size()];
        for(int i = 0; i < angles.size(); i++)
            angle_idx[i] = i;*/

        // Numero di punti in cui passare
        int k = t.size();

        std::cout << "DIMENSIONE PUNTI:" << k << std::endl;

        //Get all the permutation of angles over victims (combinations k angles over n victims)
        //vector<vector<int>> descent = getPermutation(angle_idx, te, angles.size(), k);

        //std::cout << "Discese: " << descent.size() << std::endl;

        //Foreach angles permutation
        /*for (int i = 0; i < descent.size(); i++) {
            //std::cout << "Discesa numero: " << i << std::endl;
            //Re-initialize every loop

            //Descent actual length
            double total_L = 0;*/

        rob_x = x;
        rob_y = y;
        rob_theta = 0;

        //Foreach victim to reach
        for (int a = 0; a < k; a++) {
            int id;
            Voronoi::Point p(0,0);
            double xf, yf, angle;
            tie(id, p, angle) = t[a];

            xf = p.a / 500;
            yf = p.b / 500;

            //Get the dubins curve
            Dubins dub;
            dub.setParams(rob_x, rob_y, rob_theta, xf, yf, angle, 50.0);

            pair<int, curve> ret = dub.shortest_path();
            curve cur = ret.second;

            //Adds the partial dubins length to total
            //total_L = total_L + cur.L;

            Path temp = dub.getPath(cur);
            for (const auto &point : temp.points) {
                final_path.points.push_back(point);
            }

            //Sets the actual robot position
            rob_x = xf;
            rob_y = yf;
            rob_theta = angle;
        }

            /*

            //Save all the descent length and index to classify them
            if(total_L > 0) descent_leading.emplace_back(make_pair(i, total_L));
        }*/

        //Order index by total path length
        //sort(descent_leading.begin(), descent_leading.end(), compareLength);

        //Last parameter re-initialization
        /*rob_x = x;
        rob_y = y;
        rob_theta = 0;

        //Get the best id path
        int best_id = descent_leading[0].first;*/

        //Get the path of the selected descent
        /*for (int a = 0; a < t.size(); a++) {
            double xf, yf, distance;
            xf = t[a].second.a / 500;
            yf = t[a].second.b / 500;

            Dubins dub;
            dub.setParams(rob_x, rob_y, rob_theta, xf, yf, angles[descent[best_id][a]], 10.0);

            cout << "Vittima " << a + 1 << " angolo = " << angles[descent[best_id][a]] << endl;

            pair<int, curve> ret = dub.shortest_path();
            curve cur = ret.second;

            Path temp = dub.getPath(cur);
            for (const auto &point : temp.points) {
                final_path.points.push_back(point);
            }

            rob_x = xf;
            rob_y = yf;
            rob_theta = angles[descent[best_id][a]];
        }*/

        //Add the last target, the gate
        Dubins dub;
        dub.setParams(rob_x, rob_y, rob_theta, x_gate, y_gate, M_PI / 2, 11.0);

        pair<int, curve> ret = dub.shortest_path();
        curve cur = ret.second;

        Path temp = dub.getPath(cur);
        for (const auto &point : temp.points) {
            final_path.points.push_back(point);
        }


        auto done = std::chrono::high_resolution_clock::now();

        cout << "Tempo di esecuzione: " << std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count() << endl;

        //Send the path to the robot
        path = final_path;

    }
}

