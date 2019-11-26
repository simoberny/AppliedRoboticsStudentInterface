#include <boost/config.hpp>
#include <iostream>
#include <fstream>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace std;
using namespace boost;

int main() {
    typedef adjacency_list<listS, vecS, directedS,
            no_property, property<edge_weight_t, int> > graph_t;
    typedef graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
    typedef graph_traits<graph_t>::edge_descriptor edge_descriptor;
    typedef std::pair<int, int> Edge;

    vector<pair<double, double> > node_pos = {
            make_pair(0,0), make_pair(1.3, 4.3),
            make_pair(1.1,2), make_pair(1.3, 6.3),
            make_pair(5,0), make_pair(4.3, 2.3),
            make_pair(7,2), make_pair(5.3, 1.3)};

    const int num_nodes = 8;
    Edge edge_array[] = {Edge(0, 1), Edge(1, 5), Edge(0, 2), Edge(2, 4),
                         Edge(4, 5), Edge(2, 3), Edge(5, 6), Edge(5, 7),
                         Edge(3, 7)
    };

    char name[] = "01234567";
    int weights[] = {8, 5, 7, 1, 8, 6, 4, 3, 1};
    int num_arcs = sizeof(edge_array) / sizeof(Edge);

    graph_t g(edge_array, edge_array + num_arcs, weights, num_nodes);
    property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);

    std::vector<vertex_descriptor> p(num_vertices(g));
    std::vector<int> d(num_vertices(g));
    vertex_descriptor s = vertex(0, g);

    property_map<graph_t, vertex_index_t>::type indexmap = get(vertex_index, g);

    dijkstra_shortest_paths(g, s, &p[0], &d[0], weightmap, indexmap,
                            std::less<int>(), closed_plus<int>(),
                            (std::numeric_limits<int>::max)(), 0,
                            default_dijkstra_visitor());

    std::cout << "distances and parents:" << std::endl;

    graph_traits < graph_t >::vertex_iterator vi, vend;
    for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
        std::cout << "distance(" << name[*vi] << ") = " << d[*vi] << ", ";
        std::cout << "parent(" << name[*vi] << ") = " << name[p[*vi]] << std::
        endl;
    }
    std::cout << std::endl;

    int n = 7;
    vector<int> path;
    while (n != 0) {
        path.push_back(n);
        n = p[n]; // you're one step closer to the source..
    }

    path.push_back(0);

    cout << "Cammino minimo: ";
    for(int i = 0; i < path.size(); i++){
        cout << path[i] << "(" << node_pos[path[i]].first << "," << node_pos[path[i]].second << ")";
        if(i < path.size()-1) cout << " --> ";
    }

    cout << endl;

    std::ofstream dot_file("dijkstra-eg.dot");

    dot_file << "digraph D {\n"
             << "  rankdir=LR\n"
             << "  size=\"4,3\"\n"
             << "  ratio=\"fill\"\n"
             << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";

    for(int i = 0; i < num_nodes; i++){
        dot_file << name[i]
                 << "[pos=\"" << node_pos[i].first << "," << node_pos[i].second  << "!\"]\n";
    }

    graph_traits < graph_t >::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {

        graph_traits < graph_t >::edge_descriptor e = *ei;
        graph_traits < graph_t >::vertex_descriptor u = source(e, g), v = target(e, g);


        dot_file << name[u] << " -> " << name[v]
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
    return EXIT_SUCCESS;
}