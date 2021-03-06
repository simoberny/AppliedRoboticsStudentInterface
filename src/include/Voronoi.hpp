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
const double max_threshold_dist = 0.90;

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

    int robot_pos = 0;
    int gate_pos = 0;

    /**
     * Graph structure to compute the Djistra minimal path
     */
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

        void clear_path(){
            this->shortest_path.clear();
        }

        /**
         * Initialization function
         * @param vd complete voronoi graph
         * @param merged_obstacles obstacle list
         */
        void createGraph(voronoi_diagram<double> &vd, std::vector<Polygon> &merged_obstacles) {
            // Vertex array
            int vertex_id = 0;
            for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) {
                double x = it->x() / scale;
                double y = it->y() / scale;
                if(!voronoi_match_obstacles(merged_obstacles,x,y)) {
                    this->vertex_map.emplace_back(Voronoi::Point(x, y));
                }

                vertex_id++;
            }

            // Edge array based on previous vertex
            for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
                if ((it->vertex0() != NULL) &&
                    (it->vertex1() != NULL)) {  //NOTE: se la retta va all'infinito allora vertex = NULL!!!

                    if (it->is_primary()) {
                        Voronoi::Point p1 = Voronoi::Point(it->vertex0()->x() / scale, it->vertex0()->y() / scale);
                        Voronoi::Point p2 = Voronoi::Point(it->vertex1()->x() / scale, it->vertex1()->y() / scale);

                        int pos_1 = get_pos_array(this->vertex_map, p1);
                        int pos_2 = get_pos_array(this->vertex_map, p2);

                        if (pos_1 != -1 && pos_2 != -1) {
                            //Compute the euclidean distance that is used as a weight
                            double distance = sqrt(pow(p1.a - p2.a, 2) + pow(p1.b - p2.b, 2));

                            this->weights.emplace_back(distance * 1000.0);
                            this->edge.emplace_back(Edge(pos_1, pos_2));
                        }
                    }
                }
            }

            //Initialize structure
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

        /**
         * Compute the minimal path between to graph point
         * @param p1 point numbered one
         * @param p2 point numbered two
         * @return points path
         */
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

    std::vector<std::tuple<int, Voronoi::Point, double> > graph(voronoi_diagram<double> &vd,std::vector<Polygon> merged_obstacles, const float theta, double& gate_angle, int program, std::vector<Polygon> clean_obstacles);
    
    void recover_nothing(Voronoi::Graph &myg, std::vector<std::pair<int, bool> > &fastest_path);
    void recover_all(Voronoi::Graph &myg, std::vector<std::pair<int, bool> > &recover_path);
    void fast_recover(Voronoi::Graph &myg, std::vector<std::pair<int, bool> > &recover_path);
    
    static bool voronoi_match_obstacles(std::vector<Polygon> merged_obstacles,double x,double y);
    static int get_pos_array(std::vector<Voronoi::Point> v, Voronoi::Point el);
};


#endif //TEST_VORONOI_H
