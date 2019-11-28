#include <iostream>

#include "include/boost/geometry.hpp"
#include "include/boost/geometry/geometries/box.hpp"
#include "include/boost/geometry/geometries/point_xy.hpp"
#include "include/boost/geometry/geometries/polygon.hpp"
#include "include/boost/geometry/io/wkt/wkt.hpp"



//#include "/home/osboxes/Desktop/boost_1_71_0/boost/polygon/voronoi.hpp"

#include <boost/polygon/voronoi.hpp>

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;


#include <deque>

#include <boost/foreach.hpp>

namespace geom = boost::geometry;


#include <vector>

using namespace std;

struct Point {
    int a;
    int b;

    Point(int x, int y) : a(x), b(y) {}
};

struct Segment {
    Point p0;
    Point p1;

    Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};


namespace boost {
    namespace polygon {

        template<>
        struct geometry_concept<Point> {
            typedef point_concept type;
        };

        template<>
        struct point_traits<Point> {
            typedef int coordinate_type;

            static inline coordinate_type get(
                    const Point &point, orientation_2d orient) {
                return (orient == HORIZONTAL) ? point.a : point.b;
            }
        };

        template<>
        struct geometry_concept<Segment> {
            typedef segment_concept type;
        };

        template<>
        struct segment_traits<Segment> {
            typedef int coordinate_type;
            typedef Point point_type;

            static inline point_type get(const Segment &segment, direction_1d dir) {
                return dir.to_int() ? segment.p1 : segment.p0;
            }
        };
    }  // polygon
}  // boost

int iterate_primary_edges1(const voronoi_diagram<double> &vd) {
    int result = 0;
    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
        if (it->is_primary())
            ++result;
    }
    return result;

}

int main() {

    std::vector<Point> points;
    //points.push_back(Point(0, 0));
    //points.push_back(Point(1, 6));
    std::vector<Segment> segments;
    //cubo 1
    segments.push_back(Segment(0, 0, 5, 0));
    segments.push_back(Segment(5, 0, 5, 5));
    segments.push_back(Segment(5, 5, 0, 5));
    segments.push_back(Segment(0, 5, 0, 0));


    voronoi_diagram<double> vd;
    construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);

    std::cout << "print iterate primary edge " << iterate_primary_edges1(vd);

    return 0;
}


