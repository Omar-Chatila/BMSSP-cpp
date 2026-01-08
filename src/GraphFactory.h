#ifndef GRAPH_FACTORY_H
#define GRAPH_FACTORY_H

#include <fstream>
#include <string>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <unordered_set>

#include "Graph.h"


inline Graph graph_from_csv(const char* filename) {
    std::ifstream f(filename);

    if (!f.is_open()) {
        throw std::invalid_argument("File not found!");
    }

    Graph g;
    std::string str;
    // Delimiter
    while (getline(f, str)){
        uint64_t from_id;
        uint64_t to_id;
        double weight;
        double rest;
        int res = sscanf(str.c_str(),"%lu,%lu,%lf,%lf", &from_id, &to_id, &weight, &rest);
        if (res < 3) 
            sscanf(str.c_str(),"%lu,%lu,%lf", &from_id, &to_id, &weight);
            
        g.add_vertex(from_id);
            if (weight < 0.0) continue;
        g.add_edge(from_id, to_id, weight);
    }

    f.close();
    std::cout << "Graph with " << g.size() << " vertices and " << g.edges_size() << " edges parsed" << std::endl;
    return g;
}

inline std::vector<const Vertex*> get_start_vertices(Graph& g, int num) {
    auto& vertices = g.get_vertices();
    std::random_device rd;
    std::mt19937 gen(42);
    std::uniform_int_distribution<> distrib(0, g.size() - 1);

    std::vector<const Vertex*> result;
    std::unordered_set<uint64_t> added;

    result.reserve(num);
    for (int i = 0; i < num; ++i) {
        const int index = distrib(gen);
        const Vertex* candidate = &vertices.at(index);
        while (candidate->outgoing_edges_.empty() || added.contains(candidate->id_)) {
            const int index = distrib(gen);
            candidate = &vertices.at(index);
        }
        result.push_back(candidate);
        added.emplace(candidate->id_);
    }
    return result;
}

#endif

