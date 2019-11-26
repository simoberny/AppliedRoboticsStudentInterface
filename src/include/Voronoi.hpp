//
// Created by osboxes on 11/24/19.
//

#ifndef TEST_VORONOI_H
#define TEST_VORONOI_H

#pragma once
#include <iostream>
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//#include "/home/osboxes/Desktop/boost_1_71_0/boost/polygon/voronoi.hpp"
#include <boost/polygon/voronoi.hpp>

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

class Voronoi {

public:
    struct Point {
        double a;
        double b;
        Point (int x, int y) : a(x), b(y) {}
    };

    struct Segment {
        Point p0;
        Point p1;
        Segment (int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
    };

        Voronoi();
        void calculate(const std::vector<Polygon>& obstacle_list,const Polygon& borders,const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, voronoi_diagram<double>& vd);
        void draw(const std::vector<Polygon>& obstacle_list,const Polygon& borders,const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, voronoi_diagram<double>& vd);
};


#endif //TEST_VORONOI_H
