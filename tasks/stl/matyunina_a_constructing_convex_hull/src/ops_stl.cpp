#include "stl/matyunina_a_constructing_convex_hull/include/ops_stl.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <thread>
#include <set>
#include <stack>
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
        int base = i;
        for (int j = 0; j < height_; ++j) {
          if (input_[j * width_ + base] == 1) {
            local_points[t].emplace_back(Point(i, j));
          }
        }
      }
    });
  }

  for (auto &th : threads) {
    if (th.joinable()) th.join();
  }

  size_t total_found = 0;
  for (const auto &v : local_points) total_found += v.size();
  points_.reserve(points_.size() + total_found);

  for (auto &v : local_points) {
    points_.insert(points_.end(),
                   std::make_move_iterator(v.begin()),
                   std::make_move_iterator(v.end()));
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

  for (Point& p : points_) {
    if (p.x < leftmost.x) leftmost = p;
    if (p.x > rightmost.x) rightmost = p;
  }

  std::stack<std::pair<Point, Point>> segmentStack;
  std::set<Point> hullSet;

  hullSet.insert(leftmost);
  hullSet.insert(rightmost);
  segmentStack.push({leftmost, rightmost});
  segmentStack.push({rightmost, leftmost});

  while (!segmentStack.empty()) {
    Point a = segmentStack.top().first;
    Point b = segmentStack.top().second;
    segmentStack.pop();

    double maxDistance = -1.0;
    Point farthestPoint;
    bool found = false;

    const int num_threads = ppc::util::GetPPCNumThreads();
    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    std::vector<double> local_max(num_threads, -1.0);
    std::vector<Point> local_point(num_threads);
    std::vector<bool> local_found(num_threads, false);

    for (int t = 0; t < num_threads; ++t) {
      threads.emplace_back([&, t]() {
        size_t start = (static_cast<size_t>(t) * points_.size()) / num_threads;
        size_t end   = (static_cast<size_t>(t + 1) * points_.size()) / num_threads;

        double bestDist = -1.0;
        Point bestPoint;
        bool hasPoint = false;

        for (size_t i = start; i < end; ++i) {
          Point& p = points_[i];
          if (Point::orientation(a, b, p) > 0) {
            double dist = Point::distanceToLine(a, b, p);
            if (dist > bestDist) {
              bestDist = dist;
              bestPoint = p;
              hasPoint = true;
            }
          }
        }

        local_max[t] = bestDist;
        local_point[t] = bestPoint;
        local_found[t] = hasPoint;
      });
    }

    for (auto& th : threads) th.join();

    for (int t = 0; t < num_threads; ++t) {
      if (local_found[t] && local_max[t] > maxDistance) {
        maxDistance = local_max[t];
        farthestPoint = local_point[t];
        found = true;
      }
    }

    if (found) {
      hullSet.insert(farthestPoint);

      segmentStack.push({a, farthestPoint});
      segmentStack.push({farthestPoint, b});
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
