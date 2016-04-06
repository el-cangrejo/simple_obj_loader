#include "ObjLoader.hpp"
#include "Mesh.hpp"

#include "benchmark/benchmark.h"


static void BM_ObjLoader(benchmark::State& state) {
  Mesh mesh;
  std::string filename();
  while (state.KeepRunning())
    loadObj()
}

// Register the function as a benchmark
BENCHMARK(BM_ObjLoader);

BENCHMARK_MAIN();