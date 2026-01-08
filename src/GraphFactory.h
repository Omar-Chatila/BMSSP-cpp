#ifndef GRAPH_FACTORY_H
#define GRAPH_FACTORY_H

#include <fstream>
#include <string>
#include <iostream>
#include <stdexcept>

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
        g.add_edge(from_id, to_id, weight);
    }

    f.close();
    std::cout << "Graph with " << g.size() << " vertices and " << g.edges_size() << " edges parsed" << std::endl;
    return g;
}

#endif

