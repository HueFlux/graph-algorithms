#ifndef _GRAPH_HPP_
#define _GRAPH_HPP_

#include <vector>

class Edge {
    public:
        int source;
        int destination;
        int weight;

        Edge(){}

        Edge(int src, int dest, int w) :
            source (src),
            destination (dest),
            weight (w)
        {}

        friend bool operator>(const Edge& lhs, const Edge& rhs) {
            return lhs.weight > rhs.weight;
        }
};

class Vertex {
    public:
        // Vertex identifier
        int id;
        // Vector containing all edges with vertex as source
        std::vector<Edge> adjacency_list;

        Vertex(int id, std::vector<Edge> adj_list) :
            id (id),
            adjacency_list (adj_list)
        {}
};

class Graph {
    public:
        // Vector containing all edges of graph
        std::vector<Edge> edges;
        // Vector containing all vertices of graph
        std::vector<Vertex> vertices;

        // Constructor which takes vectors containing all graph edges and vertices
        Graph(std::vector<Edge> e, std::vector<Vertex> v);
        // Constructor which takes an adjacency matrix and populates
        // the edges and vertices vectors
        Graph(const std::vector<std::vector<int> >& adjacency_matrix);
};

inline Graph::Graph(std::vector<Edge> e, std::vector<Vertex> v) :
    edges (e),
    vertices (v)
{}

Graph::Graph(const std::vector<std::vector<int> >& adjacency_matrix) {
    vertices.reserve(adjacency_matrix.size());
    for (int i = 0; i < adjacency_matrix.size(); i++) {
        std::vector<Edge> adjacency_list;

        for (int j = 0; j < adjacency_matrix[0].size(); j++) {
            int weight = adjacency_matrix[i][j];

            if (weight > 0) { // Edge exists
                Edge edge = Edge(i, j, weight);
                adjacency_list.push_back(edge);
                edges.push_back(edge);
            }
        }
        vertices.push_back(Vertex(i, adjacency_list));
    }
}

#endif
