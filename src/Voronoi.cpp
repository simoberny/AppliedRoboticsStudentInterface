//
// Created by osboxes on 11/24/19.
//

#include "include/Voronoi.hpp"

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
            typedef int coordinate_type;
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

void Voronoi::calculate(const std::vector<Polygon>& obstacle_list,const Polygon& borders,const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, voronoi_diagram<double>& vd){
    std::vector<Point> points;
    std::vector<Segment> segments;

    std::pair<double,double> gate_center = calcCentroid(gate);


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



    for (int i = 0; i < victim_list.size(); i++) {
        std::pair<double,double> victim_center = calcCentroid(victim_list[i].second);
        cv::line( image, cv::Point( victim_center.first*scala1, victim_center.second*scala1 ), cv::Point( victim_center.first*scala1+10, victim_center.second*scala1-10), cv::Scalar( 255, 127 ,0 ),  2, 1 );
        cv::line( image, cv::Point( victim_center.first*scala1, victim_center.second*scala1 ), cv::Point( victim_center.first*scala1+10, victim_center.second*scala1+10), cv::Scalar( 255, 127 ,0 ),  2, 1 );
        cv::line( image, cv::Point( victim_center.first*scala1, victim_center.second*scala1 ), cv::Point( victim_center.first*scala1-10, victim_center.second*scala1-10), cv::Scalar( 255, 127 ,0 ),  2, 1 );
        cv::line( image, cv::Point( victim_center.first*scala1, victim_center.second*scala1 ), cv::Point( victim_center.first*scala1-10, victim_center.second*scala1+10), cv::Scalar( 255, 127 ,0 ),  2, 1 );

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
                std::cout << "edge: x1: " << it->vertex0()->x() << " y1: " << it->vertex0()->y() << " \t  x2 "
                          << it->vertex1()->x()
                          << " y2: " << it->vertex1()->y() << std::endl;
                cv::line(image, cv::Point(it->vertex0()->x() , it->vertex0()->y() ),
                         cv::Point(it->vertex1()->x() , it->vertex1()->y() ), cv::Scalar(110, 220, 0), 1, 8);

            }
        }
    }

    for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) {
        std::cout << "vertex: x1: " << it->x() << " \t y1: " << it->y() << std::endl;
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
