#ifndef ALGO_SEMINAR_BENCHMARKSETUP_H
#define ALGO_SEMINAR_BENCHMARKSETUP_H

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <sys/stat.h>

#include "Graph.h"

inline std::vector<std::vector<std::string>> undirected_csv;
inline std::vector<std::vector<std::string>> directed_csv;

int run_benchmarks();


inline bool file_exists(const std::string& name) {
    const std::filesystem::path file = std::filesystem::absolute(name.c_str());
    struct stat buffer;
    std::cout << file.c_str() << std::endl;
    return (stat (file.c_str(), &buffer) == 0);
}

inline void
create_graph_csvs(const int n_start, const int n_end, const int deg_start, const int deg_end, const GraphType type) {
    if (type == GraphType::UNDIRECTED) {
        undirected_csv.resize(n_end + 1);
        for (auto& vec : undirected_csv) {
            vec.resize(deg_end + 1);
        }
    } else {
        directed_csv.resize(n_end + 1);
        for (auto& vec : directed_csv) {
            vec.resize(deg_end + 1);
        }
    }

    for (int n = n_start; n <= n_end; ++n) {
        const auto nodes = static_cast<size_t>(std::pow(2, n));
        for (int d = deg_start; d <= deg_end; ++d) {
            if (n >= 23 && d == 3) return;
            const size_t deg = static_cast<size_t>(std::pow(2, d));

            const std::string graph_name = std::string(type == GraphType::DIRECTED ? "directed_" : "undirected_") + std::to_string(nodes) + "_" + std::to_string(deg);
            auto file_name = "../resources/benchmarks/" + graph_name;
            if (type == GraphType::UNDIRECTED)
                undirected_csv[n][d] = file_name;
            else
                directed_csv[n][d] = file_name;
            if (file_exists(file_name)) {
                std::cout << graph_name << " exists -> skipping" << std::endl;
                continue;
            }

            std::string command = "python3 ../resources/gen_rand_graph.py " + graph_name + " " + std::to_string(nodes) + " " + std::to_string(deg) + " " + std::to_string(static_cast<int>(type));
            const int ret = std::system(command.c_str());
            if (ret != 0) {
                throw std::invalid_argument("exec failed");
            }
        }
    }
}

#endif //ALGO_SEMINAR_BENCHMARKSETUP_H