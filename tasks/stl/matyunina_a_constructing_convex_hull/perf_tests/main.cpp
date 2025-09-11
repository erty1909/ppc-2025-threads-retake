#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/perf/include/perf.hpp"
#include "core/task/include/task.hpp"
#include "stl/matyunina_a_constructing_convex_hull/include/ops_stl.hpp"

TEST(matyunina_a_constructing_convex_hull_stl, test_pipeline_run) {
  constexpr int count = 8000;

  std::vector<int> image(count * count, 1);

  auto task_data = std::make_shared<ppc::core::TaskData>();
  task_data->inputs.emplace_back(reinterpret_cast<uint8_t*>(image.data()));
  task_data->inputs_count.emplace_back(count);
  task_data->inputs_count.emplace_back(count);

  // Create Task
  auto constructingConvexHull =
      std::make_shared<matyunina_a_constructing_convex_hull_stl::ConstructingConvexHull>(task_data);

  // Create Perf attributes
  auto perf_attr = std::make_shared<ppc::core::PerfAttr>();
  perf_attr->num_running = 10;
  const auto t0 = std::chrono::high_resolution_clock::now();
  perf_attr->current_timer = [&] {
    auto current_time_point = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time_point - t0).count();
    return static_cast<double>(duration) * 1e-9;
  };

  // Create and init perf results
  auto perf_results = std::make_shared<ppc::core::PerfResults>();

  // Create Perf analyzer
  auto perf_analyzer = std::make_shared<ppc::core::Perf>(constructingConvexHull);
  perf_analyzer->PipelineRun(perf_attr, perf_results);
  ppc::core::Perf::PrintPerfStatistic(perf_results);

  matyunina_a_constructing_convex_hull_stl::Point* pointArray =
      reinterpret_cast<matyunina_a_constructing_convex_hull_stl::Point*>(task_data->outputs[0]);
  std::vector<matyunina_a_constructing_convex_hull_stl::Point> points(pointArray,
                                                                      pointArray + task_data->outputs_count[0]);

  std::vector<matyunina_a_constructing_convex_hull_stl::Point> ans = {
      {0, 0},
      {0, count - 1},
      {count - 1, 0},
      {count - 1, count - 1},
  };
  EXPECT_EQ(points, ans);
}

TEST(matyunina_a_constructing_convex_hull_stl, test_task_run) {
  constexpr int count = 8000;

  std::vector<int> image(count * count, 1);

  auto task_data = std::make_shared<ppc::core::TaskData>();
  task_data->inputs.emplace_back(reinterpret_cast<uint8_t*>(image.data()));
  task_data->inputs_count.emplace_back(count);
  task_data->inputs_count.emplace_back(count);

  // Create Task
  auto constructingConvexHull =
      std::make_shared<matyunina_a_constructing_convex_hull_stl::ConstructingConvexHull>(task_data);

  // Create Perf attributes
  auto perf_attr = std::make_shared<ppc::core::PerfAttr>();
  perf_attr->num_running = 10;
  const auto t0 = std::chrono::high_resolution_clock::now();
  perf_attr->current_timer = [&] {
    auto current_time_point = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time_point - t0).count();
    return static_cast<double>(duration) * 1e-9;
  };

  // Create and init perf results
  auto perf_results = std::make_shared<ppc::core::PerfResults>();

  // Create Perf analyzer
  auto perf_analyzer = std::make_shared<ppc::core::Perf>(constructingConvexHull);
  perf_analyzer->TaskRun(perf_attr, perf_results);
  ppc::core::Perf::PrintPerfStatistic(perf_results);

  matyunina_a_constructing_convex_hull_stl::Point* pointArray =
      reinterpret_cast<matyunina_a_constructing_convex_hull_stl::Point*>(task_data->outputs[0]);
  std::vector<matyunina_a_constructing_convex_hull_stl::Point> points(pointArray,
                                                                      pointArray + task_data->outputs_count[0]);

  std::vector<matyunina_a_constructing_convex_hull_stl::Point> ans = {
      {0, 0},
      {0, count - 1},
      {count - 1, 0},
      {count - 1, count - 1},
  };
  EXPECT_EQ(points, ans);
}
