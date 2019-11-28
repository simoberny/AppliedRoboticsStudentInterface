//
// Created by osboxes on 11/24/19.
//

#include "include/Voronoi.hpp"
#include <math.h>

const int scale = 500;
const int scala1 = 500;
const double scala2 = 500.0;

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

Voronoi::Voronoi() {     // Constructor

}

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



    void Voronoi::calculate(const std::vector<Polygon>& obstacle_list,const Polygon& borders,const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, voronoi_diagram<double>& vd,Voronoi::Point& robot_center,std::vector<std::pair<int,Voronoi::Point>>& victims_center,Voronoi::Point& gate_center){
    std::vector<Point> points;
    std::vector<Segment> segments;
        //rectangle_gate(borders,gate,triangle_gate);

    std::vector<std::vector<double>> triangle_robot;
    compute_triangle_robot(borders,x,y,theta,triangle_robot);

    segments.push_back(Segment(triangle_robot[0][0]*scala1, triangle_robot[0][1]*scala1, triangle_robot[1][0]*scala1, triangle_robot[1][1]*scala1));
    segments.push_back(Segment(triangle_robot[0][0]*scala1, triangle_robot[0][1]*scala1, triangle_robot[2][0]*scala1, triangle_robot[2][1]*scala1));
    robot_center.a = triangle_robot[0][0];
    robot_center.b = triangle_robot[0][1];


    std::vector<std::vector<double>> triangle_gate;
    compute_triangle_gate(borders,gate,triangle_gate);

    segments.push_back(Segment(triangle_gate[0][0]*scala1, triangle_gate[0][1]*scala1, triangle_gate[1][0]*scala1, triangle_gate[1][1]*scala1));
    segments.push_back(Segment(triangle_gate[0][0]*scala1, triangle_gate[0][1]*scala1, triangle_gate[2][0]*scala1, triangle_gate[2][1]*scala1));
    gate_center.a = triangle_gate[0][0];
    gate_center.b = triangle_gate[0][1];



    // croce sulle vittime per forzare il diagramma di voronoi a passare sopra
    for (int i = 0; i < victim_list.size(); i++) {
        std::pair<double, double> victim_center = calcCentroid(victim_list[i].second);
        victims_center.emplace_back(std::make_pair(victim_list[i].first,Voronoi::Point(victim_center.first, victim_center.second)));
        segments.push_back(
                Segment(victim_center.first * scala1, victim_center.second * scala1, victim_center.first * scala1 + 10,
                        victim_center.second * scala1 - 10));
        segments.push_back(
                Segment(victim_center.first * scala1, victim_center.second * scala1, victim_center.first * scala1 + 10,
                        victim_center.second * scala1 + 10));
        segments.push_back(
                Segment(victim_center.first * scala1, victim_center.second * scala1, victim_center.first * scala1 - 10,
                        victim_center.second * scala1 - 10));
        segments.push_back(
                Segment(victim_center.first * scala1, victim_center.second * scala1, victim_center.first * scala1 - 10,
                        victim_center.second * scala1 + 10));
    }


    for (int i = 0; i < borders.size(); i++) {
        //cv::line(image, cv::Point(borders[i].x, borders[i].y), cv::Point(5, 0), cv::Scalar(255, 255, 255), 2, 1);
        if (i <= borders.size() - 2) {
            segments.push_back(Segment(borders[i].x * scala1, borders[i].y * scala1, borders[i + 1].x * scala1,
                                       borders[i + 1].y * scala1));
        } else {
            segments.push_back(Segment(borders[i].x * scala1, borders[i].y * scala1, borders[0].x * scala1,
                                       borders[0].y * scala1));
        }
    }


    for (int i = 0; i < obstacle_list.size(); i++) {
        Polygon v = obstacle_list[i];

        for (int j = 0; j < obstacle_list[i].size(); j++) {
            if (j <= v.size() - 2) {
                //cv::line( image, cv::Point(v[j].x*500, v[j].y*500), cv::Point(v[(j+1)].x*500, v[(j+1)].y*500), cv::Scalar( 255, 255, 255 ),  2, 1 );
                segments.push_back(Segment(v[j].x * scala1, v[j].y * scala1, v[j + 1].x * scala1, v[j + 1].y * scala1));
            } else {
                //cv::line( image, cv::Point(v[j].x*500, v[j].y*500), cv::Point(v[0].x*500, v[0].y*500), cv::Scalar( 255, 255, 255 ),  2, 1 );
                segments.push_back(Segment(v[j].x * scala1, v[j].y * scala1, v[0].x * scala1, v[0].y * scala1));
            }
        }
    }


    for (int i = 0; i < matrix_size; i++) {
        segments.push_back(Segment(test_vector[i][0], test_vector[i][1], test_vector[i][2], test_vector[i][3]));
    }

    construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);

}

void Voronoi::draw(const std::vector<Polygon> &obstacle_list, const Polygon &borders,
                   const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x,
                   const float y, const float theta, voronoi_diagram<double> &vd,
                   const std::vector<std::pair<int, Voronoi::Point> > shortest) {


    cv::Mat image = cv::Mat::zeros(600, 1000, CV_8UC3);

    
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

    for(int i = 0; i < shortest.size(); i++){
        Voronoi::Point pos_node = shortest[i].second;
        cv::circle(image, cv::Point(pos_node.a, pos_node.b), 3, cv::Scalar(0, 210, 255), 3, 8, 0);
    }

    for(int i = 0; i < shortest.size() - 1; i++){
        int node = shortest[i].first;
        Voronoi::Point pos_node = shortest[i].second;
        Voronoi::Point next_node = shortest[i+1].second;

        cv::line(image, cv::Point(pos_node.a, pos_node.b),
                 cv::Point(next_node.a, next_node.b), cv::Scalar(0, 220, 220), 2, 8);
    }

    cv::imshow("Voronoi", image);
    cv::waitKey(0);

}

using namespace boost;

int get_pos_array(std::vector<Voronoi::Point> v, Voronoi::Point el)
{
    for(int i= 0; i < v.size(); i++){
        if(v[i].a == el.a && v[i].b == el.b) return i;
    }

    return -1;
}

typedef adjacency_list<listS, vecS, undirectedS,
        no_property, property<edge_weight_t, double> > graph_t;
typedef graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
typedef graph_traits<graph_t>::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

struct has_weight_greater_than {
    has_weight_greater_than(int w_, graph_t& g_) : w(w_), g(g_) { }
    bool operator()(graph_traits<graph_t>::edge_descriptor e) {
    #if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
            property_map<graph_t, edge_weight_t>::type weight = get(edge_weight, g);
        return get(weight, e) > w;
    #else
            // This version of get() breaks VC++
            return get(edge_weight, g, e) > w;
    #endif
    }
    int w;
    graph_t& g;
};

void clean_path(std::vector<Voronoi::Point> vertex, std::vector<int> &path){
    for(int i = path.size() - 1; i > 0; i--){
        //Prune dell'albero
        double dist = sqrt(pow(vertex[path[i]].a - vertex[path[i-1]].a, 2) + pow(vertex[path[i]].b - vertex[path[i-1]].b, 2));
        if(dist < 0.02){
            std::cout << "Elimino: " << path[i-1] << std::endl;
            path.erase(path.begin() + (i-1));

        }
    }
}

double get_angle(Voronoi::Point first, Voronoi::Point second, Voronoi::Point third){
    double a1 = atan2((second.b - first.b), (second.a - first.a));
    double a2 = atan2((third.b - second.b), (third.a - second.a));

    double a = a1 + (a2-a1) / 2;

    if(a < 0){
        a = 2 * M_PI + a;
    }

    std::cout << "A1: " << a1 << " ; A2:" << a2 <<   " ; Angolo di approccio: " << a <<  std::endl;
    std::cout << std::endl;

    return a;
}

bool coeff_higher(Voronoi::Point first, Voronoi::Point second, Voronoi::Point third){
    double m1 = atan((second.b - first.b) / (second.a - first.a));
    double m2 = atan((third.b - second.b) / (third.a - second.a));

    double diff = fabs(m2 - m1);

    std::cout << "P1: " << first.a << "," << first.b << std::endl;
    std::cout << "P2: " << second.a << "," << second.b << std::endl;
    std::cout << "P3: " << third.a << "," << third.b << std::endl;
    std::cout << "M1: " << m1 << " ; M2:" << m2 <<   " ; Diff: " << diff <<  std::endl;

    return diff > 0.2;
}

std::vector<std::tuple<int, Voronoi::Point, double> > Voronoi::graph(voronoi_diagram<double> &vd) {
    std::vector<Edge> edge_array;
    std::vector<double> weights;
    std::vector<Voronoi::Point> vertex_map;

    int vertex_id = 0;

    for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) {
        double x = it->x() / scala2;
        double y = it->y() / scala2;

        vertex_map.emplace_back(Voronoi::Point(x, y));

        vertex_id++;
    }

    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
        if ((it->vertex0() != NULL) &&
            (it->vertex1() != NULL)) {  //NOTE: se la retta va all'infinito allora vertex = NULL!!!

            if (it->is_primary()) {
                Voronoi::Point p1 = Voronoi::Point(it->vertex0()->x() / scala2 , it->vertex0()->y() / scala2);
                Voronoi::Point p2 = Voronoi::Point(it->vertex1()->x() / scala2, it->vertex1()->y() / scala2);

                int pos_1 = get_pos_array(vertex_map, p1);
                int pos_2 = get_pos_array(vertex_map, p2);

                if(pos_1 != -1 && pos_2 != -1){
                    double distance = sqrt(pow(p1.a - p2.a, 2) + pow(p1.b - p2.b, 2));

                    weights.emplace_back(distance * 1000.0);
                    edge_array.emplace_back(Edge(pos_1, pos_2));
                }
            }
        }
    }

    int num_nodes = vertex_map.size();
    std::cout << "Numero: " << num_nodes << std::endl;

    double arr[weights.size()];
    std::copy(weights.begin(), weights.end(), arr);

    Edge array_edge[edge_array.size()];
    std::copy(edge_array.begin(), edge_array.end(), array_edge);

    int num_arcs = sizeof(array_edge) / sizeof(Edge);

    graph_t g(array_edge, array_edge + num_arcs, arr, num_nodes);
    property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);

    std::vector<vertex_descriptor> p(num_vertices(g));
    std::vector<double> d(num_vertices(g));
    vertex_descriptor s = vertex(0, g);

    property_map<graph_t, vertex_index_t>::type indexmap = get(vertex_index, g);

    //remove_edge_if(has_weight_greater_than(10000, g), g);

    dijkstra_shortest_paths(g, s, &p[0], &d[0], weightmap, indexmap,
                            std::less<double>(), closed_plus<double>(),
                            10000, 0,
                            default_dijkstra_visitor());

    std::cout << "distances and parents:" << std::endl;
    graph_traits < graph_t >::vertex_iterator vi, vend;
    for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
        //std::cout << "distance(" << *vi << ") = " << d[*vi] << ", ";
        //std::cout << "parent(" << *vi << ") = " << p[*vi] << std::endl;
    }
    std::cout << std::endl;

    std::vector<std::tuple<int, Voronoi::Point, double> > shortest_path;

    int n = 545;
    std::vector<int> path;
    while (n != 0) {
        path.push_back(n);
        n = p[n]; // you're one step closer to the source..
    }

    path.push_back(0);

    std::cout << "Cammino minimo: ";
    for(int i = 0; i < path.size(); i++){
        std::cout << path[i] << "(" << vertex_map[path[i]].a << "," << vertex_map[path[i]].b << ")";
        if(i < path.size()-1) std::cout << " --> ";
    }

    std::cout << std::endl;

    clean_path(vertex_map, path);

    std::cout << "Cammino minimo dopo pruning: ";
    for(int i = 0; i < path.size(); i++){
        std::cout << path[i] << "(" << vertex_map[path[i]].a << "," << vertex_map[path[i]].b << ")";
        if(i < path.size()-1) std::cout << " --> ";
    }

    std::cout << std::endl;

    for(int i = path.size() - 1; i >= 0; i--){
        //Prune dell'albero
        if(i == 0){
            shortest_path.emplace_back(std::make_tuple(path[i], Voronoi::Point(vertex_map[path[i]].a * scala2, vertex_map[path[i]].b * scala2), 0));
        }else if(i == path.size() - 1) {
            shortest_path.emplace_back(std::make_tuple(path[i], Voronoi::Point(vertex_map[path[i]].a * scala2, vertex_map[path[i]].b * scala2), M_PI/2));

        }else if(i < path.size() - 1 && i != 0 ) {
            std::cout << "Archi: " << path[i + 1] << " ->" << path[i] << "->" << path[i - 1] << std::endl;

            if (coeff_higher(vertex_map[path[i + 1]], vertex_map[path[i]], vertex_map[path[i - 1]])) {
                double angle = get_angle(vertex_map[path[i + 1]], vertex_map[path[i]], vertex_map[path[i-1]]);

                shortest_path.emplace_back(std::make_tuple(path[i], Voronoi::Point(vertex_map[path[i]].a * scala2,
                                                                                   vertex_map[path[i]].b * scala2),
                                                           angle));
            }
        }
    }

    std::cout << "Numero nodi: " << shortest_path.size() << std::endl;


    std::ofstream dot_file("/tmp/dijkstra-eg.dot");

    dot_file << "digraph D {\n"
             << "  rankdir=LR\n"
             << "  size=\"70,65\"\n"
             << "  ratio=\"fill\"\n"
             << "  edge[style=\"bold\", fontsize=26]\n" << "  node[shape=\"circle\", fontsize=28]\n";

    for(int i = 0; i < num_nodes; i++){
        dot_file << i
                 << "[pos=\"" << vertex_map[i].a << "," << vertex_map[i].b  << "!\"]\n";
    }

    graph_traits < graph_t >::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
        graph_traits < graph_t >::edge_descriptor e = *ei;
        graph_traits < graph_t >::vertex_descriptor
                u = source(e, g), v = target(e, g);
        dot_file << u << " -> " << v
                 << "[label=\"" << get(weightmap, e) << "\"";


        bool shortest = false;

        for(int i = path.size() - 1; i > 0; i--){
            if(u == path[i] && v == path[i-1]){
                shortest = true;
            }
        }

        if (shortest)
            dot_file << ", color=\"black\"";
        else
            dot_file << ", color=\"grey\"";
        dot_file << "]";
    }
    dot_file << "}";

    //cv::imshow("Voronoi", image);
    //cv::waitKey(0);

    return shortest_path;
}



