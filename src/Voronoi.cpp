//
// Created by osboxes on 11/24/19.
//

#include "include/Voronoi.hpp"
#include <math.h>

const int scale = 500;
const int scala1 = 500;

namespace boost {
    namespace polygon {

        template <>
        struct geometry_concept<Voronoi::Point> {
            typedef point_concept type;
        };

        template <>
        struct point_traits<Voronoi::Point> {
            typedef int coordinate_type;

            static inline coordinate_type get(
                    const Voronoi::Point& point, orientation_2d orient) {
                return (orient == HORIZONTAL) ? point.a : point.b;
            }
        };

        template <>
        struct geometry_concept<Voronoi::Segment> {
            typedef segment_concept type;
        };

        template <>
        struct segment_traits<Voronoi::Segment> {
            typedef double coordinate_type;
            typedef Voronoi::Point point_type;

            static inline point_type get(const Voronoi::Segment& segment, direction_1d dir) {
                return dir.to_int() ? segment.p1 : segment.p0;
            }
        };
    }  // polygon
}  // boost


Voronoi::Voronoi(){     // Constructor

}

//const int test_vector[][4] = {{90,90,100,100},{100,100,110,90}};
//const int test_vector[][4] = {{50,50,70,30},{50,50,70,70}};
float test_vector[4][4];
const int matrix_size = 4;

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

    return std::make_pair(cx, cy);
}

double distance_line_point(double l1_x1, double l1_y1, double l1_x2, double l1_y2,double p_x1, double p_y1)
{

    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::linestring<point_type> linestring_type;

    linestring_type line1;
    point_type p1(p_x1,p_y1);
    line1.push_back(point_type(l1_x1,l1_y1));
    line1.push_back(point_type(l1_x2,l1_y2));
    std::cout
            << "Line-point: " << boost::geometry::distance(line1, p1) << std::endl;
    return boost::geometry::distance(line1, p1);

}
double distance_line_line(double l1_x1, double l1_y1, double l1_x2, double l1_y2, double l2_x1, double l2_y1, double l2_x2, double l2_y2)
{

    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::linestring<point_type> linestring_type;

    linestring_type line1;
    linestring_type line2;
    line1.push_back(point_type(l1_x1,l1_y1));
    line1.push_back(point_type(l1_x2,l1_y2));
    line2.push_back(point_type(l2_x1,l2_y1));
    line2.push_back(point_type(l2_x2,l2_y2));
    std::cout
            << "Line-line: " << boost::geometry::distance(line1, line2) << std::endl;
    return boost::geometry::distance(line1, line2);

}

void compute_triangle_gate(const Polygon& borders,const Polygon& gate,std::vector<std::vector<double>>& triangle_gate) {
    double triangle_lenght = 0.03;
    std::pair<double, double> gate_center = calcCentroid(gate);
    std::vector<double> v1 = {gate_center.first,gate_center.second};
    triangle_gate.emplace_back(v1);
    double distance = 10000.0;
    int index = 0;
    //trova il lato del bordo più vicino a centro del gate....
    for (int i = 0; i < borders.size(); i++) {
        //cv::line(image, cv::Point(borders[i].x, borders[i].y), cv::Point(5, 0), cv::Scalar(255, 255, 255), 2, 1);
        if (i <= borders.size() - 2) {
            double d = distance_line_point(borders[i].x , borders[i].y, borders[i + 1].x,
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

    double x1,y1;
    double x2,y2;
    if (index <= borders.size() - 2) {
        x1= borders[index].x;
        y1 = borders[index].y;
        x2= borders[index+1].x;
        y2 = borders[index+1].y;
    } else {
        x1= borders[index].x;
        y1 = borders[index].y;
        x2= borders[0].x;
        y2 = borders[0].y;
    }
    std::cout << "index border : "<<index<<" distance gate-border: " <<distance<<" gate center: "<< gate_center.first << " " << gate_center.second<<std::endl;

    //TODO: attenzione se non vengono trovati i 2 punti del triangolo con massima distanza dal bordo triangle vector non avrà 2 vettori di punti e la lettura dei valori del triangolo andrà out of boundaries killando il programma!!!
    //TODO: verificre con differenti angoli di partenza!!!S
    std::vector<std::vector<double>> cross_vertex = {{gate_center.first+triangle_lenght, gate_center.second-triangle_lenght},{gate_center.first+triangle_lenght, gate_center.second+triangle_lenght},{gate_center.first-triangle_lenght, gate_center.second-triangle_lenght},{gate_center.first-triangle_lenght, gate_center.second+triangle_lenght}};
    for(int i = 0; i< cross_vertex.size(); i++){

        if(distance_line_point(x1,y1,x2,y2,cross_vertex[i][0],cross_vertex[i][1])>distance){
            std::vector<double> v = {cross_vertex[i][0],cross_vertex[i][1]};
            triangle_gate.emplace_back(v);
            std::cout << "vertex added: x: "<<cross_vertex[i][0]<<" y: " <<cross_vertex[i][1]<<" "<<std::endl;
        }
    }
}


void compute_triangle_robot(const Polygon& borders,const float x, const float y, const float theta,std::vector<std::vector<double>>& triangle_robot){
    //angolo sul robot:
    double pi = 3.14159;
    double m = -tan(theta);
    double delta = 0.03;


    std::vector<double> v1;
    v1.emplace_back(x);
    v1.emplace_back(y);
    triangle_robot.emplace_back(v1);

    if(theta<1.57 || theta >4.7){ //primo e quarto quadrante pi/2 e 273 pi
        double m1= m+0.1;
        double m2= m-0.1;
        double q1 = y-m1*x;
        double q2 = y-m2*x;
        std::vector<double> v2 = {x+delta,m1*(x+delta)+q1};
        std::vector<double> v3 = {x+delta,m2*(x+delta)+q2};
        std::cout<< "caso1..... m1: "<< m1 <<" m2: "<< m2 <<" q1: "<< q1 <<" q2: "<< q2 <<" theta: "<<theta<<" tan: "<<m<<" p1x: "<<x+delta<<" p1y: "<<m1*(x+delta)+q1<<" p2x: "<<x+delta<<" p2y: "<<m2*(x+delta)+q2<<std::endl;
        triangle_robot.emplace_back(v2);
        triangle_robot.emplace_back(v3);
    }else{
        double m1= m+0.1;
        double m2= m-0.1;
        double q1 = y-m1*x;
        double q2 = y-m2*x;
        std::vector<double> v2 = {x-delta,m1*(x-delta)+q1};
        std::vector<double> v3 = {x-delta,m2*(x-delta)+q2};
        std::cout<< "caso2..... m1: "<< m1 <<" m2: "<< m2 <<" q1: "<< q1 <<" q2: "<< q2 <<" theta: "<<theta<<" tan: "<<m<<" p1x: "<<x+delta<<" p1y: "<<m1*(x+delta)+q1<<" p2x: "<<x+delta<<" p2y: "<<m2*(x+delta)+q2<<std::endl;
        triangle_robot.emplace_back(v2);
        triangle_robot.emplace_back(v3);
    }
}



    void Voronoi::calculate(const std::vector<Polygon>& obstacle_list,const Polygon& borders,const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, voronoi_diagram<double>& vd){
    std::vector<Point> points;
    std::vector<Segment> segments;
        //rectangle_gate(borders,gate,triangle_gate);

    std::vector<std::vector<double>> triangle_robot;
    compute_triangle_robot(borders,x,y,theta,triangle_robot);

    segments.push_back(Segment(triangle_robot[0][0]*scala1, triangle_robot[0][1]*scala1, triangle_robot[1][0]*scala1, triangle_robot[1][1]*scala1));
    segments.push_back(Segment(triangle_robot[0][0]*scala1, triangle_robot[0][1]*scala1, triangle_robot[2][0]*scala1, triangle_robot[2][1]*scala1));

    std::vector<std::vector<double>> triangle_gate;
    compute_triangle_gate(borders,gate,triangle_gate);

    segments.push_back(Segment(triangle_gate[0][0]*scala1, triangle_gate[0][1]*scala1, triangle_gate[1][0]*scala1, triangle_gate[1][1]*scala1));
    segments.push_back(Segment(triangle_gate[0][0]*scala1, triangle_gate[0][1]*scala1, triangle_gate[2][0]*scala1, triangle_gate[2][1]*scala1));


    // croce sulle vittime per forzare il diagramma di voronoi a passare sopra
    for (int i = 0; i < victim_list.size(); i++) {
        std::pair<double,double> victim_center = calcCentroid(victim_list[i].second);
        segments.push_back(Segment(victim_center.first*scala1, victim_center.second*scala1, victim_center.first*scala1+10, victim_center.second*scala1-10));
        segments.push_back(Segment(victim_center.first*scala1, victim_center.second*scala1, victim_center.first*scala1+10, victim_center.second*scala1+10));
        segments.push_back(Segment(victim_center.first*scala1, victim_center.second*scala1, victim_center.first*scala1-10, victim_center.second*scala1-10));
        segments.push_back(Segment(victim_center.first*scala1, victim_center.second*scala1, victim_center.first*scala1-10, victim_center.second*scala1+10));
    }


    for (int i = 0; i < borders.size(); i++) {
        //cv::line(image, cv::Point(borders[i].x, borders[i].y), cv::Point(5, 0), cv::Scalar(255, 255, 255), 2, 1);
        if (i <= borders.size() - 2) {
            segments.push_back(Segment(borders[i].x*scala1, borders[i].y*scala1, borders[i + 1].x*scala1, borders[i + 1].y*scala1));
        } else {
            segments.push_back(Segment(borders[i].x * scala1, borders[i].y * scala1, borders[0].x * scala1, borders[0].y * scala1));
        }
    }


    for (int i = 0; i < obstacle_list.size(); i++) {
        Polygon v = obstacle_list[i];

        for (int j = 0; j < obstacle_list[i].size(); j++) {
            if(j<=v.size()-2){
                //cv::line( image, cv::Point(v[j].x*500, v[j].y*500), cv::Point(v[(j+1)].x*500, v[(j+1)].y*500), cv::Scalar( 255, 255, 255 ),  2, 1 );
                segments.push_back(Segment(v[j].x*scala1, v[j].y*scala1, v[j+1].x*scala1, v[j+1].y*scala1));
            }else{
                //cv::line( image, cv::Point(v[j].x*500, v[j].y*500), cv::Point(v[0].x*500, v[0].y*500), cv::Scalar( 255, 255, 255 ),  2, 1 );
                segments.push_back(Segment(v[j].x*scala1, v[j].y*scala1, v[0].x*scala1, v[0].y*scala1));
            }
        }
    }


    for(int i = 0; i<matrix_size; i++){
            segments.push_back(Segment(test_vector[i][0],test_vector[i][1] ,test_vector[i][2],test_vector[i][3]));
    }

    construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);

}

void Voronoi::draw(const std::vector<Polygon>& obstacle_list,const Polygon& borders,const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, voronoi_diagram<double>& vd){

    cv::Mat image = cv::Mat::zeros( 600, 1000, CV_8UC3 );

    std::vector<std::vector<double>> triangle_gate;
    compute_triangle_gate(borders,gate,triangle_gate);

    cv::line( image, cv::Point( triangle_gate[0][0]*scale, triangle_gate[0][1]*scale ), cv::Point( triangle_gate[1][0]*scale, triangle_gate[1][1]*scale), cv::Scalar( 0,87, 	205 ),  2, 1 );
    cv::line( image, cv::Point( triangle_gate[0][0]*scale, triangle_gate[0][1]*scale ), cv::Point( triangle_gate[2][0]*scale, triangle_gate[2][1]*scale), cv::Scalar( 0,87, 	205  ),  2, 1 );
    std::cout<< "triangle gate:    x1 "<< triangle_gate[0][0]*scale <<" y1 "<< triangle_gate[0][1]*scale <<" x2 " <<triangle_gate[1][0]*scale<< " y2 "<< triangle_gate[1][1]*scale<< " x3 "<<triangle_gate[2][0]*scale <<" y3 "<<triangle_gate[1][0]*scale<<std::endl;


    std::vector<std::vector<double>> triangle_robot;
    compute_triangle_robot(borders,x,y,theta,triangle_robot);
    cv::line( image, cv::Point( triangle_robot[0][0]*scale, triangle_robot[0][1]*scale ), cv::Point( triangle_robot[1][0]*scale, triangle_robot[1][1]*scale), cv::Scalar( 255, 127 ,0 ),  2, 1 );
    cv::line( image, cv::Point( triangle_robot[0][0]*scale, triangle_robot[0][1]*scale ), cv::Point( triangle_robot[2][0]*scale, triangle_robot[2][1]*scale), cv::Scalar( 255, 127 ,0 ),  2, 1 );
    //std::cout<< "x1 "<< triangle_robot[0][0]*scale <<" y1 "<< triangle_robot[0][1]*scale <<" x2 " <<triangle_robot[1][0]*scale<< " y2 "<< triangle_robot[1][1]*scale<< " x3 "<<triangle_robot[2][0]*scale <<" y3 "<<triangle_robot[1][0]*scale<<std::endl;

    for (int i = 0; i < victim_list.size(); i++) {
        std::pair<double,double> victim_center = calcCentroid(victim_list[i].second);
        cv::line( image, cv::Point( victim_center.first*scala1, victim_center.second*scala1 ), cv::Point( victim_center.first*scala1+10, victim_center.second*scala1-10), cv::Scalar( 107,255 ,0 ),  2, 1 );
        cv::line( image, cv::Point( victim_center.first*scala1, victim_center.second*scala1 ), cv::Point( victim_center.first*scala1+10, victim_center.second*scala1+10), cv::Scalar( 107,255 ,0),  2, 1 );
        cv::line( image, cv::Point( victim_center.first*scala1, victim_center.second*scala1 ), cv::Point( victim_center.first*scala1-10, victim_center.second*scala1-10), cv::Scalar( 107,255 ,0 ),  2, 1 );
        cv::line( image, cv::Point( victim_center.first*scala1, victim_center.second*scala1 ), cv::Point( victim_center.first*scala1-10, victim_center.second*scala1+10), cv::Scalar( 107,255 ,0 ),  2, 1 );

    }


    for(int i = 0; i<matrix_size; i++){
        cv::line( image, cv::Point( test_vector[i][0], test_vector[i][1] ), cv::Point( test_vector[i][2], test_vector[i][3]), cv::Scalar( 255, 255, 255 ),  2, 1 );
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
            if(j<=v.size()-2){
                cv::line( image, cv::Point(v[j].x*scale, v[j].y*scale), cv::Point(v[(j+1)].x*scale, v[(j+1)].y*scale), cv::Scalar( 255, 255, 255 ),  2, 1 );
            }else{
                cv::line( image, cv::Point(v[j].x*scale, v[j].y*scale), cv::Point(v[0].x*scale, v[0].y*scale), cv::Scalar( 255, 255, 255 ),  2, 1 );

            }
        }

    }


    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
        if ((it->vertex0() != NULL) &&
            (it->vertex1() != NULL)) {  //NOTE: se la retta va all'infinito allora vertex = NULL!!!

            if (it->is_primary()) {
                //std::cout << "edge: x1: " << it->vertex0()->x() << " y1: " << it->vertex0()->y() << " \t  x2 " << it->vertex1()->x() << " y2: " << it->vertex1()->y() << std::endl;
                cv::line(image, cv::Point(it->vertex0()->x() , it->vertex0()->y() ),
                         cv::Point(it->vertex1()->x() , it->vertex1()->y() ), cv::Scalar(110, 220, 0), 1, 8);

            }
        }
    }

    for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) {
        //std::cout << "vertex: x1: " << it->x() << " \t y1: " << it->y() << std::endl;
        cv::circle(image, cv::Point(it->x() , it->y() ), 1, cv::Scalar(0, 0, 255), 2, 8, 0);
    }
/*
    for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin(); it != vd.cells().end(); ++it) {
        std::cout << "vertex: x1: " << it->incident_edge()-> << " \t y1: " << it->y() << std::endl;
        cv::circle(image, cv::Point(it->x() * 10, it->y() * 10), 1, cv::Scalar(0, 0, 255), 2, 8, 0);
    }
    */

    cv::imshow("Voronoi",image);
    cv::waitKey( 0 );

}