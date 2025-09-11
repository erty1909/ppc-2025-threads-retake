#include "stl/matyunina_a_constructing_convex_hull/include/ops_stl.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <set>
#include <stack>
#include <thread>
#include <mutex>
#include <vector>

#include "core/util/include/util.hpp"

bool matyunina_a_constructing_convex_hull_stl::Point::operator<(const Point& other) const {
  return (x < other.x) || (x == other.x && y < other.y);
}
bool matyunina_a_constructing_convex_hull_stl::Point::operator==(const Point& other) const {
  return x == other.x && y == other.y;
}

int matyunina_a_constructing_convex_hull_stl::Point::orientation(Point& a, Point& b, Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

double matyunina_a_constructing_convex_hull_stl::Point::distanceToLine(Point& a, Point& b, Point& c) {
  return std::abs(orientation(a, b, c));
}

double matyunina_a_constructing_convex_hull_stl::Point::distance(const Point& p1, const Point& p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool matyunina_a_constructing_convex_hull_stl::ConstructingConvexHull::PreProcessingImpl() {
  width_ = task_data->inputs_count[0];
  height_ = task_data->inputs_count[1];

  int size = width_ * height_;

  auto* in_ptr = reinterpret_cast<int*>(task_data->inputs[0]);
  input_ = std::vector<int>(in_ptr, in_ptr + size);

  return true;
}

bool matyunina_a_constructing_convex_hull_stl::ConstructingConvexHull::ValidationImpl() {
  return task_data->inputs_count[0] > 0 && task_data->inputs_count[1] > 0;
}

void matyunina_a_constructing_convex_hull_stl::ConstructingConvexHull::FindPoints() {
  points_.clear();
  const int size = width_ * height_;

  int estimated_points = 0;
  int sample = std::min(1000, size);
  for (int i = 0; i < sample; ++i) {
    if (input_[i] == 1) ++estimated_points;
  }
  double density = static_cast<double>(estimated_points) / sample;
  points_.reserve(static_cast<size_t>(size * density * 1.2));

  const int num_threads = ppc::util::GetPPCNumThreads();
  std::vector<std::thread> threads;
  threads.reserve(num_threads);

  std::vector<std::vector<Point>> local_points(num_threads);

  for (int t = 0; t < num_threads; ++t) {
    threads.emplace_back([t, num_threads, this, &local_points]() {
      int start_col = (static_cast<long long>(t) * width_) / num_threads;
      int end_col = (static_cast<long long>(t + 1) * width_) / num_threads;

      for (int i = start_col; i < end_col; ++i) {
        for (int j = 0; j < height_; ++j) {
          if (input_[j * width_ + i] == 1) {
            local_points[t].emplace_back(Point(i, j));
          }
        }
      }
    });
  }

  for (auto& th : threads) {
    if (th.joinable()) th.join();
  }

  size_t total_found = 0;
  for (const auto& v : local_points) total_found += v.size();
  points_.reserve(points_.size() + total_found);

  for (auto& v : local_points) {
    points_.insert(points_.end(), std::make_move_iterator(v.begin()), std::make_move_iterator(v.end()));
  }
}

bool matyunina_a_constructing_convex_hull_stl::ConstructingConvexHull::RunImpl() {
  FindPoints();

  if (points_.size() < 3) {
    output_ = points_;
    return true;
  }

  Point leftmost = points_[0];
  Point rightmost = points_[0];
  
  const int num_threads_extreme = ppc::util::GetPPCNumThreads();
  std::vector<std::thread> threads_extreme;
  std::vector<Point> local_leftmost(num_threads_extreme, points_[0]);
  std::vector<Point> local_rightmost(num_threads_extreme, points_[0]);

  for (int t = 0; t < num_threads_extreme; ++t) {
    threads_extreme.emplace_back([&, t]() {
      size_t start = (static_cast<size_t>(t) * points_.size()) / num_threads_extreme;
      size_t end = (static_cast<size_t>(t + 1) * points_.size()) / num_threads_extreme;

      Point local_left = points_[start];
      Point local_right = points_[start];

      for (size_t i = start; i < end; ++i) {
        if (points_[i].x < local_left.x) local_left = points_[i];
        if (points_[i].x > local_right.x) local_right = points_[i];
      }

      local_leftmost[t] = local_left;
      local_rightmost[t] = local_right;
    });
  }

  for (auto& th : threads_extreme) th.join();

  for (int t = 0; t < num_threads_extreme; ++t) {
    if (local_leftmost[t].x < leftmost.x) leftmost = local_leftmost[t];
    if (local_rightmost[t].x > rightmost.x) rightmost = local_rightmost[t];
  }

  std::stack<std::pair<Point, Point>> segmentStack;
  std::set<Point> hullSet;
  std::mutex stackMutex;
  std::mutex setMutex;

  {
    std::lock_guard<std::mutex> lock(setMutex);
    hullSet.insert(leftmost);
    hullSet.insert(rightmost);
  }

  {
    std::lock_guard<std::mutex> lock(stackMutex);
    segmentStack.push({leftmost, rightmost});
    segmentStack.push({rightmost, leftmost});
  }

  while (true) {
    std::pair<Point, Point> currentSegment;
    bool hasWork = false;

    {
      std::lock_guard<std::mutex> lock(stackMutex);
      if (!segmentStack.empty()) {
        currentSegment = segmentStack.top();
        segmentStack.pop();
        hasWork = true;
      }
    }

    if (!hasWork) break;

    Point a = currentSegment.first;
    Point b = currentSegment.second;

    double maxDistance = -1.0;
    Point farthestPoint;
    bool found = false;
    std::mutex resultMutex;

    const int num_threads = ppc::util::GetPPCNumThreads();
    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    for (int t = 0; t < num_threads; ++t) {
      threads.emplace_back([&, t]() {
        size_t start = (static_cast<size_t>(t) * points_.size()) / num_threads;
        size_t end = (static_cast<size_t>(t + 1) * points_.size()) / num_threads;

        double localMaxDist = -1.0;
        Point localBestPoint;
        bool localFound = false;

        for (size_t i = start; i < end; ++i) {
          Point& p = points_[i];
          if (Point::orientation(a, b, p) > 0) {
            double dist = Point::distanceToLine(a, b, p);
            if (dist > localMaxDist) {
              localMaxDist = dist;
              localBestPoint = p;
              localFound = true;
            }
          }
        }

        if (localFound) {
          std::lock_guard<std::mutex> lock(resultMutex);
          if (localMaxDist > maxDistance) {
            maxDistance = localMaxDist;
            farthestPoint = localBestPoint;
            found = true;
          }
        }
      });
    }

    for (auto& th : threads) th.join();

    if (found) {
      bool shouldAdd = false;
      {
        std::lock_guard<std::mutex> lock(setMutex);
        if (hullSet.find(farthestPoint) == hullSet.end()) {
          hullSet.insert(farthestPoint);
          shouldAdd = true;
        }
      }

      if (shouldAdd) {
        std::lock_guard<std::mutex> lock(stackMutex);
        segmentStack.push({a, farthestPoint});
        segmentStack.push({farthestPoint, b});
      }
    }
  }

  DeleteDublecate(hullSet);

  return true;
}

void matyunina_a_constructing_convex_hull_stl::ConstructingConvexHull::DeleteDublecate(std::set<Point>& hullSet) {
  std::vector<Point> tempHull(hullSet.begin(), hullSet.end());
  std::vector<Point> finalHull;

  Point center;
  for (const auto& p : tempHull) {
    center.x += p.x;
    center.y += p.y;
  }
  center.x /= tempHull.size();
  center.y /= tempHull.size();

  std::sort(tempHull.begin(), tempHull.end(), [&center](const Point& a, const Point& b) {
    return atan2(a.y - center.y, a.x - center.x) < atan2(b.y - center.y, b.x - center.x);
  });

  for (int i = 0; i < (int)tempHull.size(); i++) {
    Point prev = tempHull[(i - 1 + tempHull.size()) % tempHull.size()];
    Point curr = tempHull[i];
    Point next = tempHull[(i + 1) % tempHull.size()];

    if (Point::orientation(prev, curr, next) == 0) {
      double dist1 = Point::distance(prev, curr);
      double dist2 = Point::distance(curr, next);
      double dist3 = Point::distance(prev, next);

      if (std::abs(dist1 + dist2 - dist3) < 1e-9) {
        continue;
      }
    }
    finalHull.push_back(curr);
  }

  output_ = finalHull;
}

bool matyunina_a_constructing_convex_hull_stl::ConstructingConvexHull::PostProcessingImpl() {
  std::sort(output_.begin(), output_.end());

  task_data->outputs_count.push_back(output_.size());
  task_data->outputs.push_back(reinterpret_cast<uint8_t*>(output_.data()));

  return true;
}
