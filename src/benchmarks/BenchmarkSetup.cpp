#include <benchmark/benchmark.h>
#include "benchmarks/BenchmarkSetup.h"

#include "BMSSP.h"
#include "Dijkstra.h"
#include "GraphFactory.h"

constexpr int N_START_EXP = 6;
constexpr int N_END_EXP = 25;

constexpr int DEG_START_EXP = 2;
constexpr int DEG_END_EXP = 3;

static const int N_START = static_cast<int>(std::pow(2, N_START_EXP));
static const int N_END = static_cast<int>(std::pow(2, N_END_EXP));

static const int DEG_START = static_cast<int>(std::pow(2, DEG_START_EXP));
static const int DEG_END = static_cast<int>(std::pow(2, DEG_END_EXP));

static constexpr int NUM_SRCS = 5;
static constexpr int ITERATIONS = 10;

class GraphFileFixture : public benchmark::Fixture {
public:
    void SetUp(const benchmark::State &) override {
        if (undirected_csv.empty())
            create_graph_csvs(N_START_EXP, N_END_EXP, DEG_START_EXP, DEG_END_EXP, GraphType::UNDIRECTED);
        if (directed_csv.empty())
            create_graph_csvs(N_START_EXP, N_END_EXP, DEG_START_EXP, DEG_END_EXP, GraphType::DIRECTED);
    }
};

/*********************************************************************************************
 *------------------------------------------DIJKSTRA-----------------------------------------*
 *********************************************************************************************/
BENCHMARK_DEFINE_F(GraphFileFixture, Directed_Dijkstra_Execution)(benchmark::State &state) {
    const int n_index = static_cast<int>(std::log2(state.range(0)));
    const int d_index = static_cast<int>(std::log2(state.range(1)));
    Graph g = graph_from_csv(directed_csv[n_index][d_index].c_str(), GraphType::UNDIRECTED);
    if (g.empty()) {
        return state.SkipWithMessage("Graph does not exist for this (n, degree) pair");
    }
    const auto start_vertices = get_start_vertices(g, NUM_SRCS);
    for (auto _: state) {
        for (const Vertex* src : start_vertices) {
             Dijkstra dijkstra(g, src);
             auto res = dijkstra.fib_heap_run();
             benchmark::DoNotOptimize(res);
        }
        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations() * NUM_SRCS);
    state.SetComplexityN(state.range(0));
}

BENCHMARK_REGISTER_F(GraphFileFixture, Directed_Dijkstra_Execution)
     ->Name("Directed Graphs - Dijkstra Fib Heap")
     ->RangeMultiplier(2)
     ->RangePair(N_START, N_END, DEG_START, DEG_END)
     ->Unit(benchmark::kMicrosecond)
     ->Complexity(benchmark::oAuto)
     ->Iterations(ITERATIONS);


/*********************************************************************************************
 *------------------------------------------DIJKSTRA - Binary Heap-----------------------------------------*
 *********************************************************************************************/

BENCHMARK_DEFINE_F(GraphFileFixture, Directed_std_Dijkstra_Execution)(benchmark::State &state) {
    const int n_index = static_cast<int>(std::log2(state.range(0)));
    const int d_index = static_cast<int>(std::log2(state.range(1)));
    Graph g = graph_from_csv(directed_csv[n_index][d_index].c_str(), GraphType::UNDIRECTED);
    if (g.empty()) {
        return state.SkipWithMessage("Graph does not exist for this (n, degree) pair");
    }
    const auto start_vertices = get_start_vertices(g, NUM_SRCS);
    for (auto _: state) {
        for (const Vertex* src : start_vertices) {
             Dijkstra dijkstra(g, src);
             auto res = dijkstra.std_heap_run();
             benchmark::DoNotOptimize(res);
        }
        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations() * NUM_SRCS);
    state.SetComplexityN(state.range(0));
}

BENCHMARK_REGISTER_F(GraphFileFixture, Directed_std_Dijkstra_Execution)
     ->Name("Directed Graphs - Dijkstra Binary Heap")
     ->RangeMultiplier(2)
     ->RangePair(N_START, N_END, DEG_START, DEG_END)
     ->Unit(benchmark::kMicrosecond)
     ->Complexity(benchmark::oAuto)
     ->Iterations(ITERATIONS);


/*********************************************************************************************
 *------------------------------------------BMSSP--------------------------------------------*
 *********************************************************************************************/

BENCHMARK_DEFINE_F(GraphFileFixture, Directed_BMSSP_Execution)(benchmark::State &state) {
    const int n_index = static_cast<int>(std::log2(state.range(0)));
    const int d_index = static_cast<int>(std::log2(state.range(1)));
    Graph g = graph_from_csv(directed_csv[n_index][d_index].c_str(), GraphType::UNDIRECTED);
    if (g.empty()) {
        return state.SkipWithMessage("Graph does not exist for this (n, degree) pair");
    }
    const auto start_vertices = get_start_vertices(g, NUM_SRCS);
    for (auto _: state) {
        for (const Vertex* src : start_vertices) {
            BMSSP bmssp(g, src);
            auto res = bmssp.run();
            benchmark::DoNotOptimize(res);
        }
        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations() * NUM_SRCS);
    state.SetComplexityN(state.range(0));
}

BENCHMARK_REGISTER_F(GraphFileFixture, Directed_BMSSP_Execution)
        ->Name("Directed Graphs - BMSSP")
        ->RangeMultiplier(2)
        ->RangePair(N_START, N_END, DEG_START, DEG_END)
        ->Unit(benchmark::kMicrosecond)
        ->Complexity(benchmark::oAuto)
        ->Iterations(ITERATIONS);



int run_benchmarks(int argc, char** argv) {
    benchmark::MaybeReenterWithoutASLR(argc, argv);
    ::benchmark::Initialize(&argc, argv);
    if (::benchmark::ReportUnrecognizedArguments(argc, argv))
        return 1;
    ::benchmark::RunSpecifiedBenchmarks();
    ::benchmark::Shutdown();
    return 0;
}