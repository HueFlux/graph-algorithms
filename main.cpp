#include <iostream>
#include "graph.hpp"
#include <queue>
#include "disjoint_subsets.hpp"
#include <array>
#include <utility>
#include <chrono>

// Uses Prim's algorithm to construct a minimum spanning tree of graph
// Returns a vector of edges composing a minimum spanning tree of graph
std::vector<Edge> Prim(const Graph& graph);
// Uses Kruskal's algorithm to construct a minimum spanning tree of graph
// Returns a vector of edges composing a minimum spanning tree of graph
std::vector<Edge> Kruskal(const Graph& graph);
// Uses Dijkstra's algorithm to find the shortest path from vertex source
// to every other vertex in graph
// Returns a vector of arrays of the format {v, d, p} where v is another
// vertex in the graph, d is the length of the shortest path from source
// to v and p is it's penultimate vertex
std::vector<std::array<int, 3> > Dijkstra(const Graph& graph, int source);

int main() {
    std::string vertex_names = "abcdefghijklmnopqrstuvwxyz";
    // Sample adjacency matrix of weighted connected graph
    // See sample_graph.jpg for graph diagram
    std::vector<std::vector<int> > adjacency_matrix = {
        {0, 3, 5, 4, 0, 0, 0, 0, 0, 0, 0, 0},
        {3, 0, 0, 0, 3, 6, 0, 0, 0, 0, 0, 0},
        {5, 0, 0, 2, 0, 0, 4, 0, 0, 0, 0, 0},
        {4, 0, 2, 0, 1, 0, 0, 5, 0, 0, 0, 0},
        {0, 3, 0, 1, 0, 2, 0, 0, 4, 0, 0, 0},
        {0, 6, 0, 0, 2, 0, 0, 0, 0, 5, 0, 0},
        {0, 0, 4, 0, 0, 0, 0, 3, 0, 0, 6, 0},
        {0, 0, 0, 5, 0, 0, 3, 0, 6, 0, 7, 0},
        {0, 0, 0, 0, 4, 0, 0, 6, 0, 3, 0, 5},
        {0, 0, 0, 0, 0, 5, 0, 0, 3, 0, 0, 9},
        {0, 0, 0, 0, 0, 0, 6, 7, 0, 0, 0, 8},
        {0, 0, 0, 0, 0, 0, 0, 0, 5, 9, 8, 0}
    };

    Graph graph = Graph(adjacency_matrix);

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Edge> minimum_spanning_tree = Prim(graph);
    auto stop = std::chrono::high_resolution_clock::now();

    std::cout << "Minimum Spanning Tree (Prim):\n";
    for (Edge e : minimum_spanning_tree) {
        std::cout << vertex_names[e.source] << "<->"
                  << vertex_names[e.destination] << '\n';
    }
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time: " << duration.count() << " microseconds\n";

    std::cout << '\n';

    start = std::chrono::high_resolution_clock::now();
    minimum_spanning_tree = Kruskal(graph);
    stop = std::chrono::high_resolution_clock::now();

    std::cout << "Minimum Spanning Tree (Kruskal):\n";
    for (Edge e : minimum_spanning_tree) {
        std::cout << vertex_names[e.source] << "<->"
                  << vertex_names[e.destination] << '\n';
    }
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time: " << duration.count() << " microseconds" << '\n';

    std::cout << '\n';

    int source = 0;
    start = std::chrono::high_resolution_clock::now();
    std::vector<std::array<int, 3> > shortest_paths = Dijkstra(graph, source);
    stop = std::chrono::high_resolution_clock::now();

    std::cout << "Shortest Paths From " << vertex_names[source] << ":\n";
    for (int i = 1; i < shortest_paths.size(); i++) {
        std::cout << "To: " << vertex_names[shortest_paths[i][0]]
                  << " Distance: " << shortest_paths[i][1]
                  << " Predecessor: " << vertex_names[shortest_paths[i][2]]
                  << '\n';
    }
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time: " << duration.count() << " microseconds\n";

    return 0;
}

std::vector<Edge> Prim(const Graph& graph) {
    // Vector to keep track of visted vertices
    // Index corresponds to vertex id
    std::vector<bool> visited(graph.vertices.size(), false);
    std::vector<Edge> tree_edges;
    // Min-heap priority queue containing all adjacent edges to unvisited
    // vertices
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge> > new_edges;

    // Mark first vertex of graph (initial vertex) as visted
    visited[0] = true;

    // Populate priority queue with edges with initial vertex as source
    for (const Edge& edge : graph.vertices[0].adjacency_list) {
        new_edges.push(edge);
    }

    for (int i = 1; i < graph.vertices.size(); i++) {
        Edge next_edge;

        do { // Get minumum-weight edge that extends the tree
            next_edge = new_edges.top();
            new_edges.pop();
        } while (visited[next_edge.destination]);

        const Vertex& new_vertex = graph.vertices[next_edge.destination];
        visited[new_vertex.id] = true;
        tree_edges.push_back(std::move(next_edge));

        // Add all edges with new vertex as source to new_edges priority queue
        for (const Edge& edge : new_vertex.adjacency_list) {
            new_edges.push(edge);
        }

    }

    return tree_edges;
}

std::vector<Edge> Kruskal(const Graph& graph) {
    std::vector<Edge> tree_edges;
    // Min-heap priority queue used to contain all edges in graph sorted
    // by weight
    std::priority_queue<Edge,
                        std::vector<Edge>,
                        std::greater<Edge> > sorted_edges;

    // Populate priority queue with graph edges
    for (const Edge& edge : graph.edges) {
        sorted_edges.push(edge);
    }

    // DisjointSubsets object used to keep track of conneced components and
    // check for possible cycle creation during the algorithm
    DisjointSubsets subcomponents(graph.vertices.size());

    // Loop variable to count edges added to tree
    int encounter = 0;
    while (encounter < graph.vertices.size() - 1) {
        Edge current_edge = sorted_edges.top();
        sorted_edges.pop();

        // If edge does not create a cycle, add to tree
        if (subcomponents.Union(current_edge.source, current_edge.destination)) {
            tree_edges.push_back(std::move(current_edge));
            encounter++;
        }
    }

    return tree_edges;
}

std::vector<std::array<int, 3> > Dijkstra(const Graph& graph, int source) {
    struct Label {
        // Id of vertex to which Label belongs
        int vertex_id;
        // Predecessor vertex in current shortest path from source to vertex
        // Value is -1 if vertex is source
        int predecessor;
        // Length of current known shortest path from source to vertex
        int cost;

        Label(int vertex_id, int predecessor, int cost) :
            vertex_id(vertex_id),
            predecessor(predecessor),
            cost(cost)
        {}

        bool operator>(const Label& rhs) const {
            return this->cost > rhs.cost;
        }
    };
    // Vector of arrays to contain all shortest path information
    std::vector<std::array<int, 3> > shortest_paths;
    shortest_paths.reserve(graph.vertices.size());
    // Vector to keep track of visted vertices
    // Index corresponds to vertex id
    std::vector<bool> visited(graph.vertices.size(), false);
    // Min-heap priority queue used to contain Labels of all fringe vertices
    // sorted by cost
    std::priority_queue<Label,
                        std::vector<Label>,
                        std::greater<Label> > fringe_vertices;

    fringe_vertices.emplace(source, -1, 0); // Make label for source vertex

    // Loop variable to count shortest paths found
    int counter = 0;
    while (counter < graph.vertices.size()) {
        // Get label with lowest cost among fringe vertices
        Label current_label = fringe_vertices.top();
        fringe_vertices.pop();

        if (visited[current_label.vertex_id]) {
            continue;
        }
        // Mark vertex as visited
        visited[current_label.vertex_id] = true;

        // Add array containing shortest path information to return vector
        std::array<int, 3> shortest_path = {current_label.vertex_id,
                                            current_label.cost,
                                            current_label.predecessor};
        shortest_paths.push_back(std::move(shortest_path));

        // Get corresponding Vertex object in graph
        const Vertex& current_vertex = graph.vertices[current_label.vertex_id];
        // Make new labels for all adjacent vertices
        for (const Edge& edge : current_vertex.adjacency_list) {
            fringe_vertices.emplace(edge.destination,
                                    edge.source,
                                    current_label.cost + edge.weight);
        }
        counter++;
    }

    return shortest_paths;
}
