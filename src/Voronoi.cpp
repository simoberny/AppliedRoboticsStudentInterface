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

std::vector<std::vector<double>> triangle_gate;
double gate_angle;
std::vector<std::vector<double>> triangle_robot;

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
    //std::cout << "Line-point: " << boost::geometry::distance(line1, p1) << std::endl;
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
   // std::cout<< "Line-line: " << boost::geometry::distance(line1, line2) << std::endl;
    return boost::geometry::distance(line1, line2);
}

//find the points of the acute angle over the gate
void
compute_triangle_gate(const Polygon &borders, const Polygon &gate, std::vector<std::vector<double>> &triangle_gate, double& gate_angle) {


    double triangle_length = 0.05;
    std::pair<double, double> gate_center = calcCentroid(gate);
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
    /*
    std::cout << "index border : " << index << " distance gate-border: " << distance << " gate center: "
              << gate_center.first << " " << gate_center.second << std::endl;
              */

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
           // std::cout << "vertex added: x: " << cross_vertex[i][0] << " y: " << cross_vertex[i][1] << " " << std::endl;
        }
    }

    double x_center = (triangle_gate[0][0]+triangle_gate[1][0]+gate_center.first)/3;
    double y_center = (triangle_gate[0][1]+triangle_gate[1][1]+gate_center.second)/3;
    std::vector<double> v = {x_center, y_center};
    triangle_gate.emplace_back(v);



    double a1 = atan2((triangle_gate[1][1] - triangle_gate[2][1]), (triangle_gate[1][0] - triangle_gate[2][0]));
    double a2 = atan2((triangle_gate[0][1] - triangle_gate[2][1]), (triangle_gate[0][0] - triangle_gate[2][0]));

    double meta = 0;

  
   
    if(a1< M_PI/2 && a1>0 && a2> -M_PI/2 && a2<0){
        //a1 primo quadrante, a2 secondo:
   
        meta = std::fmod((((a1+a2)/2)), 2*M_PI);

    }else if(a2< M_PI/2 && a2>0 && a1> -M_PI/2 && a1<0){
        meta = std::fmod((((a1+a2)/2)), 2*M_PI);

    }else{

        if(a1<0){
            a1 = 2*M_PI+a1;
        }
        if(a2<0){
            a2 = 2*M_PI+a2;
        }
    
        if(a1 < a2){
            meta = std::fmod((((a1+a2)/2)), 2*M_PI);
        }else{
            meta = std::fmod((((a1+a2)/2)), 2*M_PI);
        }        
    }

    if(meta<0){
        meta = 2*M_PI+meta;
    }
    gate_angle = std::fmod(meta+M_PI, 2*M_PI);
    //    il robot deve entrare quindi +pigregco

    //std::cout << "trovato gate...." << gate_angle << " angolo a meta: " << meta<< " a1: "<< a1 << " a2: "<<a2<< std::endl;

}

/**
 * Find the points of the acute angle over the robot
 */
void compute_triangle_robot(const Polygon &borders, const float x, const float y, const float const_theta,
                            std::vector<std::vector<double>> &triangle_robot) {

    float theta = const_theta;

    if(theta <(M_PI/2)+0.4 && theta >(M_PI/2)-0.4){
        //angolo verticale, la tangente non si può calcolare
        theta = theta +0.4;

    }else if((-M_PI/2)-0.4 <theta && theta <(-M_PI/2)+0.4){
        //angolo verticale, la tangente non si può calcolare!
        theta = theta +0.4;
    }

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
        /*
        std::cout << "caso1..... m1: " << m1 << " m2: " << m2 << " q1: " << q1 << " q2: " << q2 << " theta: "
                  << theta
                  << " tan: " << m << " p1x: " << x + delta << " p1y: " << m1 * (x + delta) + q1 << " p2x: "
                  << x + delta << " p2y: " << m2 * (x + delta) + q2 << std::endl;
                  */
        triangle_robot.emplace_back(v2);
        triangle_robot.emplace_back(v3);
    } else {
        double m1 = m + 0.1;
        double m2 = m - 0.1;
        double q1 = y - m1 * x;
        double q2 = y - m2 * x;
        std::vector<double> v2 = {x - delta, m1 * (x - delta) + q1};
        std::vector<double> v3 = {x - delta, m2 * (x - delta) + q2};
        /*
        std::cout << "caso2..... m1: " << m1 << " m2: " << m2 << " q1: " << q1 << " q2: " << q2 << " theta: "
                  << theta
                  << " tan: " << m << " p1x: " << x + delta << " p1y: " << m1 * (x + delta) + q1 << " p2x: "
                  << x + delta << " p2y: " << m2 * (x + delta) + q2 << std::endl;
                  */
        triangle_robot.emplace_back(v2);
        triangle_robot.emplace_back(v3);
    }


    //std::cout << "Ho trovato triangolo robot!" << std::endl;
}
/**
 * find if there is a point in the obstagle polygons that is near the point (x,y)
 * @param merged_obstacles  polygon
 * @param x
 * @param y
 * @return true/false
 */
bool Voronoi::voronoi_match_obstacles(std::vector<Polygon> merged_obstacles,double x,double y){
    for (int i = 0; i < merged_obstacles.size(); i++) {
        Polygon v = merged_obstacles[i];
        for (int j = 0; j < v.size(); j++) {
            if ((fabs(x - v[j].x) < threshold_ricerca) && (fabs(y - v[j].y) < threshold_ricerca)){
                return true;
            }
        }
    }
    return false;
}
/**
 * the procedure caculate de voronoi diabram based on the obstacles, victime gate e robot position
 * @param obstacle_list
 * @param enlarged_borders
 * @param borders
 * @param victim_list
 * @param gate
 * @param x posizione robot
 * @param y pos robot
 * @param theta angolo robot
 * @param vd  voronoi diagram istance
 * @param gate_angle
 */

void Voronoi::calculate(const std::vector<Polygon> &obstacle_list, const Polygon &enlarged_borders, const Polygon &borders,
                        const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                        const float y, const float theta, voronoi_diagram<double> &vd, double& gate_angle) {

    std::vector<Point> points;
    std::vector<Segment> segments;

    compute_triangle_robot(borders, x, y, theta, triangle_robot);
    //pass the triangle over the robot to force voronoi
    segments.push_back(
            Segment(triangle_robot[0][0] * scale, triangle_robot[0][1] * scale, triangle_robot[1][0] * scale,
                    triangle_robot[1][1] * scale));
    segments.push_back(
            Segment(triangle_robot[0][0] * scale, triangle_robot[0][1] * scale, triangle_robot[2][0] * scale,
                    triangle_robot[2][1] * scale));
    this->robot_center.a = triangle_robot[0][0];
    this->robot_center.b = triangle_robot[0][1];


    compute_triangle_gate(borders, gate, triangle_gate, gate_angle);
    //pass the triangle over the gate to force voronoi

    segments.push_back(Segment(triangle_gate[2][0] * scale, triangle_gate[2][1] * scale, triangle_gate[1][0] * scale,
                               triangle_gate[1][1] * scale));
    segments.push_back(Segment(triangle_gate[2][0] * scale, triangle_gate[2][1] * scale, triangle_gate[0][0] * scale,
                               triangle_gate[0][1] * scale));
    this->gate_center.a = triangle_gate[2][0];
    this->gate_center.b = triangle_gate[2][1];

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


    for (int i = 0; i <enlarged_borders.size(); i++) {
        //cv::line(image, cv::Point(borders[i].x, borders[i].y), cv::Point(5, 0), cv::Scalar(255, 255, 255), 2, 1);
        if (i <= borders.size() - 2) {
            segments.push_back(Segment(enlarged_borders[i].x * scale, enlarged_borders[i].y * scale, enlarged_borders[i + 1].x * scale,
                                       enlarged_borders[i + 1].y * scale));
        } else {
            segments.push_back(Segment(enlarged_borders[i].x * scale, enlarged_borders[i].y * scale, enlarged_borders[0].x * scale,
                                       enlarged_borders[0].y * scale));
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

/**
 * Save the vornoid diagram on a file
 * @param obstacle_list
 * @param borders
 * @param victim_list
 * @param gate
 * @param x
 * @param y
 * @param theta
 * @param vd
 * @param shortest percorso minimo (stampato in giallo)
 * @return
 */

cv::Mat Voronoi::draw(const std::vector<Polygon> &obstacle_list, const Polygon &borders,
                   const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                   const float y, const float theta, voronoi_diagram<double> &vd,
                   const std::vector<std::tuple<int, Voronoi::Point, double> > shortest) {


    cv::Mat image = cv::Mat::zeros(600, 1000, CV_8UC3);


    //draw the red triangle over the gate
    cv::line(image, cv::Point(triangle_gate[2][0] * scale, triangle_gate[2][1] * scale),
             cv::Point(triangle_gate[1][0] * scale, triangle_gate[1][1] * scale), cv::Scalar(0, 87, 205), 2, 1);
    cv::line(image, cv::Point(triangle_gate[2][0] * scale, triangle_gate[2][1] * scale),
             cv::Point(triangle_gate[0][0] * scale, triangle_gate[0][1] * scale), cv::Scalar(0, 87, 205), 2, 1);
    /*
    std::cout << "triangle gate:    x1 " << triangle_gate[0][0] * scale << " y1 " << triangle_gate[0][1] * scale
              << " x2 " << triangle_gate[1][0] * scale << " y2 " << triangle_gate[1][1] * scale << " x3 "
              << triangle_gate[2][0] * scale << " y3 " << triangle_gate[1][0] * scale << std::endl;
              */

    //compute the blue triangle over the robot
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
}

using namespace boost;

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

    dot_file << "}";
}

/**
 * Return the vertex position given a point (coordinates) element
 * @param v Array of vertex index
 * @param el element to find
 * @return position index
 */

int Voronoi::get_pos_array(std::vector<Voronoi::Point> v, Voronoi::Point el) {
    for (int i = 0; i < v.size(); i++) {
        if ((fabs(el.a - v[i].a) < threshold_ricerca) && (fabs(el.b - v[i].b) < threshold_ricerca)) return i;
    }

    return -1;
}

bool is_colliding(std::vector<Polygon> merged_obstacles, Linestring ls){
    for(auto &poly : merged_obstacles){
        PolygonCollision obstacles;

        for (int j = 0; j < poly.size(); j++) {
            boost::geometry::append(obstacles.outer(), PointCollision(poly[j].x, poly[j].y));
        }

        std::vector<PointCollision> result;
        bool ret = boost::geometry::intersection(ls,obstacles,result);

        //std::cout << "Dimensione insertezione: " << result.size() << std::endl;

        if(result.size() > 0) return true;
    }

    return false;
}

void clean_path_2(std::vector<Voronoi::Point> vertex, std::vector<std::pair<int, bool> > &path, std::vector<Polygon> merged_obstacles){
    int i = 1;
    do{
        Linestring ls;
        boost::geometry::append(ls, PointCollision(vertex[path[i-1].first].a, vertex[path[i-1].first].b));
        boost::geometry::append(ls, PointCollision(vertex[path[i+1].first].a, vertex[path[i+1].first].b));

        double dist = sqrt(
                pow(vertex[path[i-1].first].a - vertex[path[i+1].first].a, 2) +
                pow(vertex[path[i-1].first].b - vertex[path[i+1].first].b, 2));

        if(is_colliding(merged_obstacles, ls)){
            //std::cout << "Is colliding " << path[i].first << std::endl;
            i++;
        }else{
            if (!path[i].second && dist < max_threshold_dist) {
                path.erase(path.begin() + (i));
            }else{
                i++;
            }
        }
    }while(i < path.size() - 1);
}

/**
 * Prune the minimum path deleting the node to close to each other
 * @param vertex array of all vertex
 * @param path Minimum path
 */
void clean_path(std::vector<Voronoi::Point> vertex, std::vector<std::pair<int, bool> > &path) {
    for (int i = path.size() - 1; i > 0; i--) {
        //Prune dell'albero
        double dist = sqrt(
                pow(vertex[path[i].first].a - vertex[path[i - 1].first].a, 2) +
                pow(vertex[path[i].first].b - vertex[path[i - 1].first].b, 2));

        if (dist < threshold_dist) {
            if (!path[i - 1].second) {
                path.erase(path.begin() + (i - 1));
            }else{
                if(!path[i].second){
                    path.erase(path.begin() + (i));
                }
            }
        }
    }
}

/**
 * Get the internal angle between two vector angle
 */
double angolo_interno(double angle1, double meta){
    if(angle1-meta >=0 && angle1-meta < M_PI)
        return angle1-meta;
    else if(angle1-meta <0 && meta-angle1 < M_PI)
        return meta-angle1;
    else if(angle1-meta >=0 && angle1-meta > M_PI)
        return 2*M_PI-(angle1-meta);
    else if(angle1-meta <0 && meta-angle1 > M_PI)
        return 2*M_PI-(meta-angle1);
    return meta;
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

    if(a1<0){
        a1 = 2*M_PI+a1;
    }
    if(a2<0){
        a2 = 2*M_PI+a2;
    }

    if(a1 < a2){
        meta = std::fmod((((a1+a2)/2) + M_PI/2), 2*M_PI);
    }else{
        meta = std::fmod((((a1+a2)/2) - M_PI/2), 2*M_PI);
    }

    if(meta<0)
        meta = 2*M_PI+meta;

    if(angolo_interno(a1, a2) < 0.20){
        std::cout << "Angolo stretto, considero meta! " << std::endl;
        return meta;
    }

    double a = meta;
    double per = (d1 > d2) ? 1 - d2/d1 : 1 - d1/d2;

    // Compute the two possible line enhancing the angle in the two side
    double a_minus = std::fmod(meta - (angolo_interno(a2,meta) * per), 2*M_PI);
    double a_plus = std::fmod(meta + (angolo_interno(a2,meta) * per), 2*M_PI);

    // Compute the internal angle based on the previous value
    double angle_a1_minus = angolo_interno(a2,a_minus);
    double angle_a1_plus = angolo_interno(a2,a_plus);

    if(d1 > d2){
        if(angle_a1_plus > angle_a1_minus){
            a = a_minus;
        }else{
            a = a_plus;
        }
    }else if (d2 > d1){
        if(angle_a1_plus > angle_a1_minus){
            a = a_plus;
        }else{
            a = a_minus;
        }
    }

    a = std::fmod(a, 2*M_PI);

    std::cout << "L1: " << d1 << " ; L2:" << d2 << std::endl;
    std::cout << "A1: " << a1 << " ; A2:" << a2  << "; Angolo metà:"  << meta << "; Angolo di approccio: " << a << std::endl;
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

    return diff > threshold_angle;
}

/**
 * Comparator for victim number
 */
bool compare_victim_number(std::pair<int, Voronoi::Point> v1, std::pair<int, Voronoi::Point> v2) {
    return (v1.first > v2.first);
}

/**
 * Main mission that save all the victim in numeric order
 * @param myg
 * @param recover_path
 */
void Voronoi::recover_all(Graph &myg, std::vector<std::pair<int, bool>> &recover_path){
    // Compute minimum path in piece through djjkstra reverse flow
    int pos = this->gate_pos;

    for (int a = 0; a < this->victims_center.size(); a++) {
        int victim_node_pos = get_pos_array(myg.vertex_map, this->victims_center[a].second);

        recover_path = myg.add_piece_path(victim_node_pos, pos);
        pos = victim_node_pos;
    }

    recover_path = myg.add_piece_path(this->robot_pos, pos);
}

/**
 * Mission that go from robot position to gate without saving any victims
 * @param myg
 * @param fastest_path
 */
void Voronoi::recover_nothing(Graph &myg, std::vector<std::pair<int, bool>> &fastest_path){
    // Compute minimum path in piece through djjkstra reverse flow
    fastest_path = myg.add_piece_path(this->robot_pos, this->gate_pos);
}

/**
 * Mission that take the fastest path and check if there are near victim that could be save without losing to much time
 * @param myg Graph
 * @param recover_path Returning path
 */
void Voronoi::fast_recover(Graph &myg, std::vector<std::pair<int, bool>> &recover_path){
    // Compute minimum path in piece through djjkstra reverse flow
    int pos = this->gate_pos;
    std::vector<std::pair<int, bool> > fastest_path = myg.add_piece_path(this->robot_pos, pos);
    clean_path(myg.vertex_map, fastest_path);

    myg.clear_path();

    std::vector<int> victim_added;

    // Last hop to the robot_po
    for (int i = 0; i < fastest_path.size() - 1; i++) {
        double x1 = myg.vertex_map[fastest_path[i].first].a;
        double y1 = myg.vertex_map[fastest_path[i].first].b;
        double x2 = myg.vertex_map[fastest_path[i+1].first].a;
        double y2 = myg.vertex_map[fastest_path[i+1].first].b;

        for (int a = 0; a < this->victims_center.size(); a++) {
            
            if (std::find(victim_added.begin(), victim_added.end(), this->victims_center[a].first) == victim_added.end()){
                double d = distance_line_point(x1, y1, x2, y2, this->victims_center[a].second.a, this->victims_center[a].second.b);

                if(d < 0.18){ // 18cm
                    int victim_node_pos = get_pos_array(myg.vertex_map, this->victims_center[a].second);

                    recover_path = myg.add_piece_path(victim_node_pos, pos);
                    pos = victim_node_pos;

                    victim_added.emplace_back(this->victims_center[a].first);
                }
            }
        }
    }

    recover_path = myg.add_piece_path(this->robot_pos, pos);
}

/**
 * Generate graph from voronoi points and calculate the minimum path from robot to gate through the victims
 * @param vd Voronoi diagram
 * @return Minimum point path including approach angle
 */
std::vector<std::tuple<int, Voronoi::Point, double> > Voronoi::graph(voronoi_diagram<double> &vd,std::vector<Polygon> merged_obstacles, const float theta, double& gate_angle, int program, std::vector<Polygon> clean_obstacles) {
    //Graph struct
    Graph myg;
    // Initialize graph
    myg.createGraph(vd, merged_obstacles);

    // Get key position based on vertex pos
    this->robot_pos = get_pos_array(myg.vertex_map, this->robot_center);
    this->gate_pos = get_pos_array(myg.vertex_map, this->gate_center);

    // Sort the victim based on number recognition
    sort(this->victims_center.begin(), this->victims_center.end(), compare_victim_number);

    // Get index of victims vertex
    std::vector<int> victim_pos;

    for (int i = 0; i < this->victims_center.size(); i++)
        victim_pos.emplace_back(get_pos_array(myg.vertex_map, this->victims_center[i].second));

    std::vector<std::pair<int, bool> > path;

    // Compute robot mission based on the configuration
    switch(program){
        case 1:
            recover_nothing(myg, path);
            break;
        case 2:
            recover_all(myg, path);
            break;
        case 3: 
            fast_recover(myg, path);
            break;
        default: 
            recover_all(myg, path);
    }

    // Graph pruning in two level of depth
    clean_path(myg.vertex_map, path);
    clean_path_2(myg.vertex_map, path, clean_obstacles);

    // Return shortest path structure
    std::vector<std::tuple<int, Voronoi::Point, double> > shortest_path;

    // Loop on path points (Reverse order)
    for (int i = path.size() - 1; i >= 0; i--) {
        if (i == 0) { // Add the gate pos
            shortest_path.emplace_back(
                    std::make_tuple(path[i].first, Voronoi::Point(myg.vertex_map[path[i].first].a * scale,
                                                                  myg.vertex_map[path[i].first].b * scale), gate_angle));
        } else if (i == path.size() - 1) { // Add the robot pos
            shortest_path.emplace_back(
                    std::make_tuple(path[i].first, Voronoi::Point(myg.vertex_map[path[i].first].a * scale,
                                                                  myg.vertex_map[path[i].first].b * scale), theta));
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

    // After pruning, I compute the real angle of approach minus first and last node
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