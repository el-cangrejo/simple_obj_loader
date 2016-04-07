#include "../include/ObjLoader.hpp"
#include "../include/Mesh.hpp"

#include "benchmark/benchmark.h"


static void BenchMark_ObjLoader(benchmark::State& state) {
  Mesh mesh;
  std::string filename("../../data/bunny.obj");
  while (state.KeepRunning())
    loadObj(filename, mesh);
}

static void BenchMark_Vector(benchmark::State& state) {
  std::vector<int> v;
  while (state.KeepRunning())
  	v.push_back(1);  
}

// Register the function as a benchmark
BENCHMARK(BenchMark_ObjLoader);

//BENCHMARK(BenchMark_Vector);

BENCHMARK_MAIN();