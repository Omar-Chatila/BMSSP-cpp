#include <benchmark/benchmark.h>
#include "benchmarks/BenchmarkSetup.h"

#include "BMSSP.h"
#include "Dijkstra.h"
#include "GraphFactory.h"

constexpr int N_START_EXP = 7;
constexpr int N_END_EXP = 17;

constexpr int DEG_START_EXP = 1;
constexpr int DEG_END_EXP = 6;

static const int N_START = static_cast<int>(std::pow(2, N_START_EXP));
static const int N_END = static_cast<int>(std::pow(2, N_END_EXP));

static const int DEG_START = static_cast<int>(std::pow(2, DEG_START_EXP));
static const int DEG_END = static_cast<int>(std::pow(2, DEG_END_EXP));

static constexpr int NUM_SRCS = 10;
static constexpr int ITERATIONS = 5;

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

BENCHMARK_DEFINE_F(GraphFileFixture, Undirected_Dijkstra_Execution)(benchmark::State &state) {
    const int n_index = std::log2(state.range(0));
    const int d_index = std::log2(state.range(1));
    Graph g = graph_from_csv(undirected_csv[n_index][d_index].c_str(), GraphType::UNDIRECTED);
    if (g.empty()) {
        return state.SkipWithMessage("Graph does not exist for this (n, degree) pair");
    }
    const auto start_vertices = get_start_vertices(g, NUM_SRCS);
    for (auto _: state) {
        for (const Vertex* src : start_vertices) {
            Dijkstra dijkstra(g, src);
            auto res = dijkstra.run();
            benchmark::DoNotOptimize(res);
        }
        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations() * NUM_SRCS);
    state.SetComplexityN(state.range(0));
}

BENCHMARK_REGISTER_F(GraphFileFixture, Undirected_Dijkstra_Execution)
        ->Name("Undirected Graphs - Dijkstra")
        ->RangeMultiplier(2)
        ->RangePair(N_START, N_END, DEG_START, DEG_END)
        ->Unit(benchmark::kMicrosecond)
        ->Complexity(benchmark::oAuto)
        ->Iterations(ITERATIONS);

BENCHMARK_DEFINE_F(GraphFileFixture, Directed_Dijkstra_Execution)(benchmark::State &state) {
    const int n_index = std::log2(state.range(0));
    const int d_index = std::log2(state.range(1));
    Graph g = graph_from_csv(directed_csv[n_index][d_index].c_str(), GraphType::UNDIRECTED);
    if (g.empty()) {
        return state.SkipWithMessage("Graph does not exist for this (n, degree) pair");
    }
    const auto start_vertices = get_start_vertices(g, NUM_SRCS);
    for (auto _: state) {
        for (const Vertex* src : start_vertices) {
            Dijkstra dijkstra(g, src);
            auto res = dijkstra.run();
            benchmark::DoNotOptimize(res);
        }
        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations() * NUM_SRCS);
    state.SetComplexityN(state.range(0));
}

BENCHMARK_REGISTER_F(GraphFileFixture, Directed_Dijkstra_Execution)
        ->Name("Directed Graphs - Dijkstra")
        ->RangeMultiplier(2)
        ->RangePair(N_START, N_END, DEG_START, DEG_END)
        ->Unit(benchmark::kMicrosecond)
        ->Complexity(benchmark::oAuto)
        ->Iterations(ITERATIONS);

/*********************************************************************************************
 *------------------------------------------BMSSP--------------------------------------------*
 *********************************************************************************************/

BENCHMARK_DEFINE_F(GraphFileFixture, Undirected_BMSSP_Execution)(benchmark::State &state) {
    const int n_index = std::log2(state.range(0));
    const int d_index = std::log2(state.range(1));
    Graph g = graph_from_csv(undirected_csv[n_index][d_index].c_str(), GraphType::UNDIRECTED);
    if (g.empty()) {
        return state.SkipWithMessage("Graph does not exist for this (n, degree) pair");
    }
    const auto start_vertices = get_start_vertices(g, NUM_SRCS);
    for (auto _: state) {
        for (const Vertex* src : start_vertices) {
            BMSSP bmssp(g, src);
            auto res = bmssp.run();
            if (bmssp.has_exec_failed()) {
                return state.SkipWithError("Execution failed");
            }
            benchmark::DoNotOptimize(res);
        }
        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations() * NUM_SRCS);
    state.SetComplexityN(state.range(0));
}

BENCHMARK_REGISTER_F(GraphFileFixture, Undirected_BMSSP_Execution)
        ->Name("Undirected Graphs - BMSSP")
        ->RangeMultiplier(2)
        ->RangePair(N_START, N_END, DEG_START, DEG_END)
        ->Unit(benchmark::kMicrosecond)
        ->Complexity(benchmark::oAuto)
        ->Iterations(ITERATIONS);

BENCHMARK_DEFINE_F(GraphFileFixture, Directed_BMSSP_Execution)(benchmark::State &state) {
    const int n_index = std::log2(state.range(0));
    const int d_index = std::log2(state.range(1));
    Graph g = graph_from_csv(directed_csv[n_index][d_index].c_str(), GraphType::UNDIRECTED);
    if (g.empty()) {
        return state.SkipWithMessage("Graph does not exist for this (n, degree) pair");
    }
    const auto start_vertices = get_start_vertices(g, NUM_SRCS);
    for (auto _: state) {
        for (const Vertex* src : start_vertices) {
            BMSSP bmssp(g, src);
            auto res = bmssp.run();
            if (bmssp.has_exec_failed()) {
                return state.SkipWithError("Execution failed");
            }
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


int run_benchmarks() {
    char arg0_default[] = "benchmark";
    char *args_default = arg0_default;

    auto argc = 1;
    const auto argv = &args_default;

    benchmark::MaybeReenterWithoutASLR(argc, argv);
    ::benchmark::Initialize(&argc, argv);
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
    ::benchmark::RunSpecifiedBenchmarks();
    ::benchmark::Shutdown();
    return 0;
}