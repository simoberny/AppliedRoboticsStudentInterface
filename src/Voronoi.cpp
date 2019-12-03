//
// Created by osboxes on 11/24/19.
//

#include "include/Voronoi.hpp"
#include <math.h>

namespace boost {
    namespace polygon {

        template<>
        struct geometry_concept<Voronoi::Point> {
            typedef point_concept type;
        };

        template<>
        struct point_traits<Voronoi::Point> {
            typedef double coordinate_type;

            static inline coordinate_type get(
                    const Voronoi::Point &point, orientation_2d orient) {
                return (orient == HORIZONTAL) ? point.a : point.b;
            }
        };

        template<>
        struct geometry_concept<Voronoi::Segment> {
            typedef segment_concept type;
        };

        template<>
        struct segment_traits<Voronoi::Segment> {
            typedef double coordinate_type;
            typedef Voronoi::Point point_type;

            static inline point_type get(const Voronoi::Segment &segment, direction_1d dir) {
                return dir.to_int() ? segment.p1 : segment.p0;
            }
        };
    }  // polygon
}  // boost

Voronoi::Voronoi() {   } // Constructor

//const int test_vector[][4] = {{90,90,100,100},{100,100,110,90}};
//const int test_vector[][4] = {{50,50,70,30},{50,50,70,70}};
float test_vector[4][4];
const int matrix_size = 4;

std::pair<double, double> calcCentroid(const Polygon &p) {
    double max_x = 0, max_y = 0, min_x = 1000, min_y = 1000;

    for (auto a : p) {
        if (a.x > max_x) max_x = a.x;
        if (a.x < min_x) min_x = a.x;
        if (a.y > max_y) max_y = a.y;
        if (a.y < min_y) min_y = a.y;
    }

    double cx = max_x - ((max_x - min_x) / 2);
    double cy = max_y - ((max_y - min_y) / 2);

    return std::make_pair(cx, cy);
}

double distance_line_point(double l1_x1, double l1_y1, double l1_x2, double l1_y2, double p_x1, double p_y1) {

    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::linestring<point_type> linestring_type;

    linestring_type line1;
    point_type p1(p_x1, p_y1);
    line1.push_back(point_type(l1_x1, l1_y1));
    line1.push_back(point_type(l1_x2, l1_y2));
    std::cout
            << "Line-point: " << boost::geometry::distance(line1, p1) << std::endl;
    return boost::geometry::distance(line1, p1);

}

double
distance_line_line(double l1_x1, double l1_y1, double l1_x2, double l1_y2, double l2_x1, double l2_y1, double l2_x2,
                   double l2_y2) {

    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::linestring<point_type> linestring_type;

    linestring_type line1;
    linestring_type line2;
    line1.push_back(point_type(l1_x1, l1_y1));
    line1.push_back(point_type(l1_x2, l1_y2));
    line2.push_back(point_type(l2_x1, l2_y1));
    line2.push_back(point_type(l2_x2, l2_y2));
    std::cout
            << "Line-line: " << boost::geometry::distance(line1, line2) << std::endl;
    return boost::geometry::distance(line1, line2);

}

void
compute_triangle_gate(const Polygon &borders, const Polygon &gate, std::vector<std::vector<double>> &triangle_gate) {
    double triangle_length = 0.03;
    std::pair<double, double> gate_center = calcCentroid(gate);
    std::vector<double> v1 = {gate_center.first, gate_center.second};
    triangle_gate.emplace_back(v1);
    double distance = 10000.0;
    int index = 0;
    //trova il lato del bordo più vicino a centro del gate....
    for (int i = 0; i < borders.size(); i++) {
        //cv::line(image, cv::Point(borders[i].x, borders[i].y), cv::Point(5, 0), cv::Scalar(255, 255, 255), 2, 1);
        if (i <= borders.size() - 2) {
            double d = distance_line_point(borders[i].x, borders[i].y, borders[i + 1].x,
                                           borders[i + 1].y, gate_center.first,
                                           gate_center.second);
            if (d < distance) {
                index = i;
                distance = d;
            }
        } else {
            double d = distance_line_point(borders[i].x, borders[i].y, borders[0].x,
                                           borders[0].y, gate_center.first,
                                           gate_center.second);
            if (d < distance) {
                index = i;
                distance = d;
            }
        }
    }

    double x1, y1;
    double x2, y2;
    if (index <= borders.size() - 2) {
        x1 = borders[index].x;
        y1 = borders[index].y;
        x2 = borders[index + 1].x;
        y2 = borders[index + 1].y;
    } else {
        x1 = borders[index].x;
        y1 = borders[index].y;
        x2 = borders[0].x;
        y2 = borders[0].y;
    }
    std::cout << "index border : " << index << " distance gate-border: " << distance << " gate center: "
              << gate_center.first << " " << gate_center.second << std::endl;

    //TODO: attenzione se non vengono trovati i 2 punti del triangolo con massima distanza dal bordo triangle vector non avrà 2 vettori di punti e la lettura dei valori del triangolo andrà out of boundaries killando il programma!!!
    //TODO: verificre con differenti angoli di partenza!!!S
    std::vector<std::vector<double>> cross_vertex = {{gate_center.first + triangle_length,
                                                                                           gate_center.second -
                                                                                           triangle_length},
                                                     {gate_center.first + triangle_length, gate_center.second +
                                                                                           triangle_length},
                                                     {gate_center.first - triangle_length, gate_center.second -
                                                                                           triangle_length},
                                                     {gate_center.first - triangle_length, gate_center.second +
                                                                                           triangle_length}};
    for (int i = 0; i < cross_vertex.size(); i++) {

        if (distance_line_point(x1, y1, x2, y2, cross_vertex[i][0], cross_vertex[i][1]) > distance) {
            std::vector<double> v = {cross_vertex[i][0], cross_vertex[i][1]};
            triangle_gate.emplace_back(v);
            std::cout << "vertex added: x: " << cross_vertex[i][0] << " y: " << cross_vertex[i][1] << " " << std::endl;
        }
    }
}


void compute_triangle_robot(const Polygon &borders, const float x, const float y, const float theta,
                            std::vector<std::vector<double>> &triangle_robot) {
    //angolo sul robot:
    double pi = 3.14159;
    double m = -tan(theta);
    double delta = 0.03;


    std::vector<double> v1;
    v1.emplace_back(x);
    v1.emplace_back(y);
    triangle_robot.emplace_back(v1);

    if (theta < 1.57 || theta > 4.7) { //primo e quarto quadrante pi/2 e 273 pi
        double m1 = m + 0.1;
        double m2 = m - 0.1;
        double q1 = y - m1 * x;
        double q2 = y - m2 * x;
        std::vector<double> v2 = {x + delta, m1 * (x + delta) + q1};
        std::vector<double> v3 = {x + delta, m2 * (x + delta) + q2};
        std::cout << "caso1..... m1: " << m1 << " m2: " << m2 << " q1: " << q1 << " q2: " << q2 << " theta: " << theta
                  << " tan: " << m << " p1x: " << x + delta << " p1y: " << m1 * (x + delta) + q1 << " p2x: "
                  << x + delta << " p2y: " << m2 * (x + delta) + q2 << std::endl;
        triangle_robot.emplace_back(v2);
        triangle_robot.emplace_back(v3);
    } else {
        double m1 = m + 0.1;
        double m2 = m - 0.1;
        double q1 = y - m1 * x;
        double q2 = y - m2 * x;
        std::vector<double> v2 = {x - delta, m1 * (x - delta) + q1};
        std::vector<double> v3 = {x - delta, m2 * (x - delta) + q2};
        std::cout << "caso2..... m1: " << m1 << " m2: " << m2 << " q1: " << q1 << " q2: " << q2 << " theta: " << theta
                  << " tan: " << m << " p1x: " << x + delta << " p1y: " << m1 * (x + delta) + q1 << " p2x: "
                  << x + delta << " p2y: " << m2 * (x + delta) + q2 << std::endl;
        triangle_robot.emplace_back(v2);
        triangle_robot.emplace_back(v3);
    }
}


void Voronoi::calculate(const std::vector<Polygon> &obstacle_list, const Polygon &borders,
                        const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                        const float y, const float theta, voronoi_diagram<double> &vd) {

    std::vector<Point> points;
    std::vector<Segment> segments;
    //rectangle_gate(borders,gate,triangle_gate);

    std::vector<std::vector<double>> triangle_robot;
    compute_triangle_robot(borders, x, y, theta, triangle_robot);

    segments.push_back(
            Segment(triangle_robot[0][0] * scale, triangle_robot[0][1] * scale, triangle_robot[1][0] * scale,
                    triangle_robot[1][1] * scale));
    segments.push_back(
            Segment(triangle_robot[0][0] * scale, triangle_robot[0][1] * scale, triangle_robot[2][0] * scale,
                    triangle_robot[2][1] * scale));
    this->robot_center.a = triangle_robot[0][0];
    this->robot_center.b = triangle_robot[0][1];


    std::vector<std::vector<double>> triangle_gate;
    compute_triangle_gate(borders, gate, triangle_gate);

    segments.push_back(Segment(triangle_gate[0][0] * scale, triangle_gate[0][1] * scale, triangle_gate[1][0] * scale,
                               triangle_gate[1][1] * scale));
    segments.push_back(Segment(triangle_gate[0][0] * scale, triangle_gate[0][1] * scale, triangle_gate[2][0] * scale,
                               triangle_gate[2][1] * scale));
    this->gate_center.a = triangle_gate[0][0];
    this->gate_center.b = triangle_gate[0][1];



    // croce sulle vittime per forzare il diagramma di voronoi a passare sopra
    for (int i = 0; i < victim_list.size(); i++) {
        std::pair<double, double> victim_center = calcCentroid(victim_list[i].second);
        this->victims_center.emplace_back(
                std::make_pair(victim_list[i].first, Voronoi::Point(victim_center.first, victim_center.second)));
        segments.push_back(
                Segment(victim_center.first * scale, victim_center.second * scale, victim_center.first * scale + 10,
                        victim_center.second * scale - 10));
        segments.push_back(
                Segment(victim_center.first * scale, victim_center.second * scale, victim_center.first * scale + 10,
                        victim_center.second * scale + 10));
        segments.push_back(
                Segment(victim_center.first * scale, victim_center.second * scale, victim_center.first * scale - 10,
                        victim_center.second * scale - 10));
        segments.push_back(
                Segment(victim_center.first * scale, victim_center.second * scale, victim_center.first * scale - 10,
                        victim_center.second * scale + 10));
    }


    for (int i = 0; i < borders.size(); i++) {
        //cv::line(image, cv::Point(borders[i].x, borders[i].y), cv::Point(5, 0), cv::Scalar(255, 255, 255), 2, 1);
        if (i <= borders.size() - 2) {
            segments.push_back(Segment(borders[i].x * scale, borders[i].y * scale, borders[i + 1].x * scale,
                                       borders[i + 1].y * scale));
        } else {
            segments.push_back(Segment(borders[i].x * scale, borders[i].y * scale, borders[0].x * scale,
                                       borders[0].y * scale));
        }
    }


    for (int i = 0; i < obstacle_list.size(); i++) {
        Polygon v = obstacle_list[i];

        for (int j = 0; j < obstacle_list[i].size(); j++) {
            if (j <= v.size() - 2) {
                //cv::line( image, cv::Point(v[j].x*500, v[j].y*500), cv::Point(v[(j+1)].x*500, v[(j+1)].y*500), cv::Scalar( 255, 255, 255 ),  2, 1 );
                segments.push_back(Segment(v[j].x * scale, v[j].y * scale, v[j + 1].x * scale, v[j + 1].y * scale));
            } else {
                //cv::line( image, cv::Point(v[j].x*500, v[j].y*500), cv::Point(v[0].x*500, v[0].y*500), cv::Scalar( 255, 255, 255 ),  2, 1 );
                segments.push_back(Segment(v[j].x * scale, v[j].y * scale, v[0].x * scale, v[0].y * scale));
            }
        }
    }


    for (int i = 0; i < matrix_size; i++) {
        segments.push_back(Segment(test_vector[i][0], test_vector[i][1], test_vector[i][2], test_vector[i][3]));
    }

    construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);



}

bool voronoi_match_obstacles(std::vector<Polygon> merged_obstacles,double x,double y){
    for (int i = 0; i < merged_obstacles.size(); i++) {
        Polygon v = merged_obstacles[i];
        for (int j = 0; j < v.size(); j++) {
            if ((fabs(x - v[j].x) < threshold_ricerca) && (fabs(y - v[j].y) < threshold_ricerca)){
                //std::cout<<"elimon un vertie:  x: "<<x<<" y: "<<y<<" vicino al obstacle: x: "<<v[j].x<<"v[j].y "<<v[j].y<<std::endl;
                return true;
            }
        }
    }
    return false;
}

cv::Mat Voronoi::draw(const std::vector<Polygon> &obstacle_list, const Polygon &borders,
                   const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                   const float y, const float theta, voronoi_diagram<double> &vd,
                   const std::vector<std::tuple<int, Voronoi::Point, double> > shortest) {


    cv::Mat image = cv::Mat::zeros(600, 1000, CV_8UC3);

    std::vector<std::vector<double>> triangle_gate;
    compute_triangle_gate(borders, gate, triangle_gate);

    cv::line(image, cv::Point(triangle_gate[0][0] * scale, triangle_gate[0][1] * scale),
             cv::Point(triangle_gate[1][0] * scale, triangle_gate[1][1] * scale), cv::Scalar(0, 87, 205), 2, 1);
    cv::line(image, cv::Point(triangle_gate[0][0] * scale, triangle_gate[0][1] * scale),
             cv::Point(triangle_gate[2][0] * scale, triangle_gate[2][1] * scale), cv::Scalar(0, 87, 205), 2, 1);
    std::cout << "triangle gate:    x1 " << triangle_gate[0][0] * scale << " y1 " << triangle_gate[0][1] * scale
              << " x2 " << triangle_gate[1][0] * scale << " y2 " << triangle_gate[1][1] * scale << " x3 "
              << triangle_gate[2][0] * scale << " y3 " << triangle_gate[1][0] * scale << std::endl;

    std::vector<std::vector<double>> triangle_robot;
    compute_triangle_robot(borders, x, y, theta, triangle_robot);
    cv::line(image, cv::Point(triangle_robot[0][0] * scale, triangle_robot[0][1] * scale),
             cv::Point(triangle_robot[1][0] * scale, triangle_robot[1][1] * scale), cv::Scalar(255, 127, 0), 2, 1);
    cv::line(image, cv::Point(triangle_robot[0][0] * scale, triangle_robot[0][1] * scale),
             cv::Point(triangle_robot[2][0] * scale, triangle_robot[2][1] * scale), cv::Scalar(255, 127, 0), 2, 1);
    //std::cout<< "x1 "<< triangle_robot[0][0]*scale <<" y1 "<< triangle_robot[0][1]*scale <<" x2 " <<triangle_robot[1][0]*scale<< " y2 "<< triangle_robot[1][1]*scale<< " x3 "<<triangle_robot[2][0]*scale <<" y3 "<<triangle_robot[1][0]*scale<<std::endl;

    for (int i = 0; i < victim_list.size(); i++) {
        std::pair<double, double> victim_center = calcCentroid(victim_list[i].second);
        cv::line(image, cv::Point(victim_center.first * scale, victim_center.second * scale),
                 cv::Point(victim_center.first * scale + 10, victim_center.second * scale - 10),
                 cv::Scalar(107, 255, 0), 2, 1);
        cv::line(image, cv::Point(victim_center.first * scale, victim_center.second * scale),
                 cv::Point(victim_center.first * scale + 10, victim_center.second * scale + 10),
                 cv::Scalar(107, 255, 0), 2, 1);
        cv::line(image, cv::Point(victim_center.first * scale, victim_center.second * scale),
                 cv::Point(victim_center.first * scale - 10, victim_center.second * scale - 10),
                 cv::Scalar(107, 255, 0), 2, 1);
        cv::line(image, cv::Point(victim_center.first * scale, victim_center.second * scale),
                 cv::Point(victim_center.first * scale - 10, victim_center.second * scale + 10),
                 cv::Scalar(107, 255, 0), 2, 1);

    }


    for (int i = 0; i < matrix_size; i++) {
        cv::line(image, cv::Point(test_vector[i][0], test_vector[i][1]),
                 cv::Point(test_vector[i][2], test_vector[i][3]), cv::Scalar(255, 255, 255), 2, 1);
    }

    for (int i = 0; i < borders.size(); i++) {
        if (i <= borders.size() - 2) {
            cv::line(image, cv::Point(borders[i].x * scale, borders[i].y * scale),
                     cv::Point(borders[i + 1].x * scale, borders[i + 1].y * scale), cv::Scalar(255, 0, 255), 2, 1);
        } else {
            cv::line(image, cv::Point(cv::Point(borders[i].x * scale, borders[i].y * scale)),
                     cv::Point(cv::Point(borders[0].x * scale, borders[0].y * scale)), cv::Scalar(255, 0, 255), 2, 1);
        }
    }


    for (int i = 0; i < obstacle_list.size(); i++) {
        Polygon v = obstacle_list[i];

        for (int j = 0; j < obstacle_list[i].size(); j++) {
            //segments.push_back(Segment(v[j].x, v[j].y, v[j+1%obstacle_list[i].size()].x, v[j+1%obstacle_list[i].size()].y));
            if (j <= v.size() - 2) {
                cv::line(image, cv::Point(v[j].x * scale, v[j].y * scale),
                         cv::Point(v[(j + 1)].x * scale, v[(j + 1)].y * scale), cv::Scalar(255, 255, 255), 2, 1);

            } else {
                cv::line(image, cv::Point(v[j].x * scale, v[j].y * scale), cv::Point(v[0].x * scale, v[0].y * scale),
                         cv::Scalar(255, 255, 255), 2, 1);

            }
        }

    }


    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
        if ((it->vertex0() != NULL) &&
            (it->vertex1() != NULL)) {  //NOTE: se la retta va all'infinito allora vertex = NULL!!!

            if (it->is_primary()) {
                //std::cout << "edge: x1: " << it->vertex0()->x() << " y1: " << it->vertex0()->y() << " \t  x2 " << it->vertex1()->x() << " y2: " << it->vertex1()->y() << std::endl;
                double x = it->vertex0()->x() / scale;
                double y = it->vertex0()->y() / scale;
                double x1 = it->vertex1()->x() / scale;
                double y1 = it->vertex1()->y() / scale;
                if(!voronoi_match_obstacles(obstacle_list,x,y)&&!voronoi_match_obstacles(obstacle_list,x1,y1)) {
                    cv::line(image, cv::Point(it->vertex0()->x(), it->vertex0()->y()),
                            cv::Point(it->vertex1()->x(), it->vertex1()->y()), cv::Scalar(110, 220, 0), 1, 8);


                }

            }
        }
    }

    for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) {
        //std::cout << "vertex: x1: " << it->x() << " \t y1: " << it->y() << std::endl;
        double x = it->x() / scale;
        double y = it->y() / scale;
        if(!voronoi_match_obstacles(obstacle_list,x,y)) {
            cv::circle(image, cv::Point(it->x(), it->y()), 1, cv::Scalar(0, 0, 255), 2, 8, 0);


        }

    }

/*
    for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin(); it != vd.cells().end(); ++it) {
        std::cout << "vertex: x1: " << it->incident_edge()-> << " \t y1: " << it->y() << std::endl;
        cv::circle(image, cv::Point(it->x() * 10, it->y() * 10), 1, cv::Scalar(0, 0, 255), 2, 8, 0);
    }
    */

    for (int i = 0; i < shortest.size(); i++) {
        Voronoi::Point pos_node = get<1>(shortest[i]);
        cv::circle(image, cv::Point(pos_node.a, pos_node.b), 3, cv::Scalar(0, 210, 255), 3, 8, 0);
    }

    for (int i = 0; i < shortest.size() - 1; i++) {
        int node = get<0>(shortest[i]);;
        Voronoi::Point pos_node = get<1>(shortest[i]);
        Voronoi::Point next_node = get<1>(shortest[i + 1]);

        cv::line(image, cv::Point(pos_node.a, pos_node.b),
                 cv::Point(next_node.a, next_node.b), cv::Scalar(0, 220, 220), 2, 8);
    }

    return image;
    //cv::startWindowThread();
    //cv::imshow("Voronoi", image);
    //cv::waitKey(0);
}

using namespace boost;

/**
 * Return the vertex position given a point (coordinates) element
 * @param v Array of vertex index
 * @param el element to find
 * @return position index
 */

int get_pos_array(std::vector<Voronoi::Point> v, Voronoi::Point el) {
    for (int i = 0; i < v.size(); i++) {
        if ((fabs(el.a - v[i].a) < threshold_ricerca) && (fabs(el.b - v[i].b) < threshold_ricerca)) return i;
    }

    return -1;
}

/**
 * Prune the minimum path deleting the node to close to each other
 * @param vertex array of all vertex
 * @param path Minimum path
 */
void clean_path(std::vector<Voronoi::Point> vertex, std::vector<std::pair<int, bool> > &path) {
    std::cout << "Elimino: ";
    for (int i = path.size() - 1; i > 0; i--) {
        //Prune dell'albero
        double dist = sqrt(
                pow(vertex[path[i].first].a - vertex[path[i - 1].first].a, 2) +
                pow(vertex[path[i].first].b - vertex[path[i - 1].first].b, 2));

        if (dist < threshold_dist) {
            if (!path[i - 1].second) {
                std::cout << path[i - 1].first << ",";
                path.erase(path.begin() + (i - 1));
            }else{
                if(!path[i].second){
                    std::cout << path[i].first << ",";
                    path.erase(path.begin() + (i));
                }
            }
        }
    }
    std::cout << std::endl;
}

/**
 * Get the approach angle of a point
 */
double get_angle(Voronoi::Point first, Voronoi::Point second, Voronoi::Point third) {
    double d1 = sqrt(
            pow(second.a - first.a, 2) +
            pow(second.b - first.b, 2));

    double d2 = sqrt(
            pow(third.a - second.a, 2) +
            pow(third.b - second.b, 2));

    double a1 = atan2((first.b - second.b), (first.a - second.a));
    double a2 = atan2((third.b - second.b), (third.a - second.a));

    double meta = 0;

    if(a1 < 0 && a2 < 0){
        if(fabs(a1) > fabs(a2)){
            meta = ((M_PI - fabs(a2-a1)) / 2) + a2;
        }else{
            meta = a2 + (((M_PI - fabs(a2-a1)) / 2) + (a2 + M_PI))  ;
        }
    }else{
        if(a1 > a2){
            meta = a2 - (M_PI - fabs(a2-a1)) / 2;
        }else{
            meta = a2 + (M_PI - fabs(a2-a1)) / 2;
        }
    }

    double a = meta;

    if(a1 < a2){
        if(d1 > d2){
            double per = d2/d1;
            a = meta * per;
        }else if (d2 > d1){
            double per = 1- d1/d2;
            a = meta + fabs(a2-a1) * per;
        }
    }else{
        if(d1 > d2){
            double per = 1- d2/d1;
            a = meta + fabs(a2-a1) * per;
        }else if (d2 > d1){
            double per = d1/d2;
            a = meta * per;
        }
    }


    std::cout << "L1: " << d1 << " ; L2:" << d2 << std::endl;
    std::cout << "A1: " << a1 << " ; A2:" << a2  << " ; Diff:"  << a2-a1 << " ; Angolo metà:"  << meta << "; Angolo di approccio: " << a << std::endl;
    std::cout << std::endl;

    return a;
}

/**
 * Boolean function to find the pairs of segments with enough angle to be considered
 * @param first Point
 * @param second Point
 * @param third Point
 * @return true if the two segment as a difference angle higher than a thresh
 */
bool coeff_higher(Voronoi::Point first, Voronoi::Point second, Voronoi::Point third) {
    double m1 = atan2((second.b - first.b), (second.a - first.a));
    double m2 = atan2((third.b - second.b), (third.a - second.a));

    double diff = fabs(m2 - m1);

    /*std::cout << "P1: " << first.a << "," << first.b << std::endl;
    std::cout << "P2: " << second.a << "," << second.b << std::endl;
    std::cout << "P3: " << third.a << "," << third.b << std::endl;*/
    std::cout << "M1: " << m1 << " ; M2:" << m2 << " ; Diff: " << diff << std::endl;

    return diff > threshold_angle;
}

/**
 * Print the minimum path
 */
void print_path(std::vector<Voronoi::Point> vertex, std::vector<int> &path) {
    std::cout << "Cammino minimo dopo pruning: ";
    for (int i = 0; i < path.size(); i++) {
        std::cout << path[i] << "(" << vertex[path[i]].a << "," << vertex[path[i]].b << ")";
        if (i < path.size() - 1) std::cout << " --> ";
    }

    std::cout << std::endl;
}

/**
 * Procedure to save the dot file for the graph
 * @param myg Graph strcture
 * @param path Minimum path
 */
void print_dot(Voronoi::Graph myg, std::vector<std::tuple<int, Voronoi::Point, double> > &path) {
    std::ofstream dot_file("/tmp/dijkstra-eg.dot");

    dot_file << "digraph D {\n"
             << "  rankdir=LR\n"
             << "  size=\"70,65\"\n"
             << "  ratio=\"fill\"\n"
             << "  edge[style=\"bold\", fontsize=26]\n" << "  node[shape=\"circle\", fontsize=28]\n";

    for (int i = 0; i < path.size(); i++) {
        dot_file << get<0>(path[i])
                 << "[pos=\"" << get<1>(path[i]).a/500 << "," << get<1>(path[i]).b/500 << "!\"]\n";
    }

    for (int i = path.size() - 1; i > 0; i--) {
        dot_file << get<0>(path[i]) << " -> " << get<0>(path[i-1])
                 << "[label=\"" << get<2>(path[i]) << "\"";
        dot_file << ", color=\"black\"";
        dot_file << "]";
    }

    graph_traits<graph_t>::edge_iterator ei, ei_end;
    /*for (tie(ei, ei_end) = edges(myg.g); ei != ei_end; ++ei) {
        graph_traits<graph_t>::edge_descriptor e = *ei;
        graph_traits<graph_t>::vertex_descriptor
                u = source(e, myg.g), v = target(e, myg.g);


        bool shortest = false;

        //std::cout << "Graph " << u << " : " << v << std::endl;

        for (int i = path.size() - 1; i > 0; i--) {
            if ((u == path[i].first && v == path[i - 1].first) ||
                (v == path[i].first && u == path[i - 1].first)) {
                std::cout << "Trovato " << path[i].first << " : " << path[i - 1].first << std::endl;

                shortest = true;
            }
        }

        if (shortest){
            dot_file << u << " -> " << v
                     << "[label=\"" << get(myg.weightmap, e) << "\"";
            dot_file << ", color=\"black\"";
            dot_file << "]";
        }

    }*/
    dot_file << "}";
}

/**
 * Comparator for victim number
 */
bool compare_victim_number(std::pair<int, Voronoi::Point> v1, std::pair<int, Voronoi::Point> v2) {
    return (v1.first > v2.first);
}



/**
 * Generate graph from voronoi points and calculate the minimum path from robot to gate through the victims
 * @param vd Voronoi diagram
 * @return Minimum point path including approach angle
 */
std::vector<std::tuple<int, Voronoi::Point, double> > Voronoi::graph(voronoi_diagram<double> &vd,std::vector<Polygon> merged_obstacles) {
    //Graph struct
    Graph myg;

    // Fill vertex array
    int vertex_id = 0;
    for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) {
        double x = it->x() / scale;
        double y = it->y() / scale;
        if(!voronoi_match_obstacles(merged_obstacles,x,y)) {
            myg.vertex_map.emplace_back(Voronoi::Point(x, y));
        }

        vertex_id++;
    }

    // Fill the edge array based on previous vertex
    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
        if ((it->vertex0() != NULL) &&
            (it->vertex1() != NULL)) {  //NOTE: se la retta va all'infinito allora vertex = NULL!!!

            if (it->is_primary()) {
                Voronoi::Point p1 = Voronoi::Point(it->vertex0()->x() / scale, it->vertex0()->y() / scale);
                Voronoi::Point p2 = Voronoi::Point(it->vertex1()->x() / scale, it->vertex1()->y() / scale);

                int pos_1 = get_pos_array(myg.vertex_map, p1);
                int pos_2 = get_pos_array(myg.vertex_map, p2);

                if (pos_1 != -1 && pos_2 != -1) {
                    //Compute the euclidean distance that is used as a weight
                    double distance = sqrt(pow(p1.a - p2.a, 2) + pow(p1.b - p2.b, 2));

                    myg.weights.emplace_back(distance * 1000.0);
                    myg.edge.emplace_back(Edge(pos_1, pos_2));
                }
            }
        }
    }

    // Get key position based on vertex pos
    int robot_pos = get_pos_array(myg.vertex_map, this->robot_center);
    int gate_pos = get_pos_array(myg.vertex_map, this->gate_center);

    std::vector<int> victim_pos;

    // Sort the victim based on number recognition
    sort(this->victims_center.begin(), this->victims_center.end(), compare_victim_number);

    // Get index of victims vertex
    for (int i = 0; i < this->victims_center.size(); i++)
        victim_pos.emplace_back(get_pos_array(myg.vertex_map, this->victims_center[i].second));


    // Compute Graph
    int num_nodes = myg.getVertexSize();
    myg.createGraph();

    // Compute minimum path in piece through djjkstra reverse flow
    std::vector<std::pair<int, bool> > path;
    int pos = gate_pos;

    for (int i = 0; i < victim_pos.size(); i++) {
        path = myg.add_piece_path(victim_pos[i], pos);
        pos = victim_pos[i];
    }

    // Last hope to the robot_pos
    path = myg.add_piece_path(robot_pos, pos);

    // Clean the noise and useless point in the graph based on euclidean distance
    clean_path(myg.vertex_map, path);
    //print_path(myg.vertex_map, path);

    // Return shortest path structure
    std::vector<std::tuple<int, Voronoi::Point, double> > shortest_path;

    // Loop on path points (Reverse order)
    for (int i = path.size() - 1; i >= 0; i--) {
        if (i == 0) { // Add the gate pos
            shortest_path.emplace_back(
                    std::make_tuple(path[i].first, Voronoi::Point(myg.vertex_map[path[i].first].a * scale,
                                                                  myg.vertex_map[path[i].first].b * scale), M_PI / 2));
        } else if (i == path.size() - 1) { // Add the robot pos
            shortest_path.emplace_back(
                    std::make_tuple(path[i].first, Voronoi::Point(myg.vertex_map[path[i].first].a * scale,
                                                                  myg.vertex_map[path[i].first].b * scale), 0));
        } else if (i < path.size() - 1 && i != 0) { // For every other point
            if (path[i].second) { // If it's a key point, add it without any doubt
                shortest_path.emplace_back(
                        std::make_tuple(path[i].first, Voronoi::Point(myg.vertex_map[path[i].first].a * scale,
                                                                      myg.vertex_map[path[i].first].b * scale), 0));

            } else if (coeff_higher(myg.vertex_map[path[i + 1].first],
                                    myg.vertex_map[path[i].first],
                                    myg.vertex_map[path[i - 1].first])) { // Otherwise if the angle reach a thresh
                shortest_path.emplace_back(
                        std::make_tuple(path[i].first, Voronoi::Point(myg.vertex_map[path[i].first].a * scale,
                                                                      myg.vertex_map[path[i].first].b * scale), 0));
            }

            // If two segment is close to a line there is no sense in considering it
        }
    }

    // After pruning, i compute the real angle of approach minus first and last node
    for(int i = 1; i < shortest_path.size() - 1; i++){
        std::cout << "Archi: " << get<0>(shortest_path[i-1]) << " ->" << get<0>(shortest_path[i]) << "->" << get<0>(shortest_path[i+1]) << std::endl;

        // Calculate the angle difference between the two segment
        double angle = get_angle(get<1>(shortest_path[i-1]), get<1>(shortest_path[i]),
                                 get<1>(shortest_path[i+1]));


        get<2>(shortest_path[i]) = angle;
    }

    print_dot(myg, shortest_path);
    return shortest_path;
}