#include <iostream>

#include "include/boost/geometry.hpp"
#include "include/boost/geometry/geometries/box.hpp"
#include "include/boost/geometry/geometries/point_xy.hpp"
#include "include/boost/geometry/geometries/polygon.hpp"
#include "include/boost/geometry/io/wkt/wkt.hpp"

#include <deque>

#include "include/find_collision.hpp"

Find_collision::Find_collision() {     // Constructor

}

namespace geom = boost::geometry;

typedef geom::model::point<double, 2, geom::cs::cartesian> point_t;
typedef geom::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
//typedef geom::model::linestring<point_t> linestring_t;
typedef boost::geometry::model::d2::point_xy<double> Point1;
typedef boost::geometry::model::linestring<Point1> Linestring;


bool
Find_collision::robot_obstacles_intersection(const std::vector<Polygon> &obstacle_list, const float x, const float y,
                                             const float theta) {

    bool result = false;
    polygon robot;
    //Linestring ls1;
    //geom::append(ls1, Point1(0.0, 0.0));
    //geom::append(ls1, Point1(1.0, 1.0));
    geom::append(robot.outer(), point_t(0.043, 0.085));
    geom::append(robot.outer(), point_t(0.04, 0.08 + 0.1));
    geom::append(robot.outer(), point_t(0.04 + 0.1, 0.08 + 0.1));
    geom::append(robot.outer(), point_t(0.04 + 0.1, 0.08));

    std::cout << "area quadrato  " << boost::geometry::area(robot) << std::endl;

    std::cout << "obstacle lists size: " << obstacle_list.size() << std::endl;

    for (int i = 0; i < obstacle_list.size(); i++) {
        polygon obstacles;
        Polygon v = obstacle_list[i];

        for (int j = 0; j < obstacle_list[i].size(); j++) {
//std::cout  << "punti poligono: " << v[j].x <<"  "<< std::endl;
            geom::append(obstacles.outer(), point_t(v[j].x, v[j].y));
        }
        //std::cout << " area " << boost::geometry::area(obstacles) << std::endl;
        std::deque<polygon> output;
        result = result || (boost::geometry::intersection(obstacles, robot, output));

        int k = 0;
        std::cout << " \t AREA intersezione :" << std::endl;

    }

    return result;

}


