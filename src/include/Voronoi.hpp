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
#include <fstream>
#include <vector>
#include <math.h>
#include <tuple>

#include "boost/graph/dijkstra_shortest_paths.hpp"
#include "boost/graph/graph_traits.hpp"
#include "boost/graph/adjacency_list.hpp"

#include "boost/polygon/voronoi.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
        boost::no_property, boost::property<boost::edge_weight_t, double> > graph_t;
typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<graph_t>::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

typedef boost::geometry::model::d2::point_xy<double> PointCollision;
typedef boost::geometry::model::linestring<PointCollision> Linestring;
typedef boost::geometry::model::polygon<PointCollision> PolygonCollision;

const double threshold_ricerca = 0.01;

const double threshold_angle = 0.22;
const double threshold_dist = 0.14;
const double max_threshold_dist = 0.50;

static double scale = 500.0;

class Voronoi {
public:
    struct Point {
        double a;
        double b;

        Point(){}
        Point(double x, double y) : a(x), b(y) {}
    };

    struct Segment {
        Point p0;
        Point p1;

        Segment(double x1, double y1, double x2, double y2) : p0(x1, y1), p1(x2, y2) {}
    };

    std::vector<std::pair<int, Voronoi::Point>> victims_center;
    Voronoi::Point robot_center;
    Voronoi::Point gate_center;

    struct Graph {
        std::vector<std::pair<int, bool> > shortest_path;

        //Graph definition
        std::vector<Edge> edge;
        std::vector<double> weights;
        std::vector<Voronoi::Point> vertex_map;

        //Boost definition
        graph_t g;
        boost::property_map<graph_t, boost::edge_weight_t>::type weightmap;
        std::vector<vertex_descriptor> p;
        std::vector<double> d;
        vertex_descriptor initial;
        boost::property_map<graph_t, boost::vertex_index_t>::type indexmap;

        int getVertexSize() {
            return vertex_map.size();
        }

        void createGraph() {
            double arr[this->weights.size()];
            std::copy(this->weights.begin(), this->weights.end(), arr);

            Edge array_edge[this->edge.size()];
            std::copy(this->edge.begin(), this->edge.end(), array_edge);

            int num_arcs = sizeof(array_edge) / sizeof(Edge);

            graph_t temp(array_edge, array_edge + num_arcs, arr, this->getVertexSize());
            this->g.copy_impl(temp);
            this->weightmap = boost::get(boost::edge_weight, this->g);

            this->p.reserve(boost::num_vertices(this->g));
            this->d.reserve(boost::num_vertices(this->g));

            this->indexmap = boost::get(boost::vertex_index, this->g);
        }

        std::vector<std::pair<int, bool> > add_piece_path(int p1, int p2) {
            vertex_descriptor s = boost::vertex(p1, this->g);

            dijkstra_shortest_paths(this->g, s, &this->p[0], &this->d[0], this->weightmap, this->indexmap,
                                    std::less<double>(), boost::closed_plus<double>(),
                                    10000, 0,
                                    boost::default_dijkstra_visitor());

            int to = p2;

            while (to != p1) {
                if(!this->shortest_path.size()){
                    this->shortest_path.push_back(std::make_pair(to, true));
                }else{
                    this->shortest_path.push_back(std::make_pair(to, false));
                }

                to = p[to]; // you're one step closer to the source..
            }

            this->shortest_path.push_back(std::make_pair(p1, true));

            return this->shortest_path;
        }
    };

    Voronoi();

    void calculate(const std::vector<Polygon> &obstacle_list, const Polygon &enlarged_borders, const Polygon &borders,
                   const std::vector<std::pair<int, Polygon>> &victim_list,
                   const Polygon &gate, const float x, const float y, const float theta, voronoi_diagram<double> &vd,double& gate_angle);

    cv::Mat draw(const std::vector<Polygon> &obstacle_list, const Polygon &borders,
              const std::vector<std::pair<int, Polygon>> &victim_list,
              const Polygon &gate, const float x, const float y, const float theta, voronoi_diagram<double> &vd,
              const std::vector<std::tuple<int, Voronoi::Point, double> > te);

    std::vector<std::tuple<int, Voronoi::Point, double> > graph(voronoi_diagram<double> &vd,std::vector<Polygon> merged_obstacles, const float theta, double& gate_angle);
};


#endif //TEST_VORONOI_H
