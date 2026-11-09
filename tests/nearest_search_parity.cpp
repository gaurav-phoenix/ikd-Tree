#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <new>
#include <random>
#include <stdexcept>
#include <thread>
#include <vector>
#include "ikd_Tree.h"

namespace {
thread_local bool track_allocations = false;
thread_local std::size_t allocation_count = 0;
}

void *operator new(std::size_t size)
{
    if (track_allocations) ++allocation_count;
    void *value = std::malloc(size ? size : 1);
    if (!value) throw std::bad_alloc();
    return value;
}
void *operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void *value) noexcept { std::free(value); }
void operator delete[](void *value) noexcept { std::free(value); }
void operator delete(void *value, std::size_t) noexcept { std::free(value); }
void operator delete[](void *value, std::size_t) noexcept { std::free(value); }

using Point = pcl::PointXYZI;
using Tree = KD_TREE<Point>;
using Points = Tree::PointVector;

Point point(float x, float y, float z, float intensity = 0)
{
    Point result;
    result.x = x; result.y = y; result.z = z; result.intensity = intensity;
    return result;
}

bool same_float(float a, float b)
{
    return std::memcmp(&a, &b, sizeof(float)) == 0;
}

bool same_points(const Points &a, const Points &b)
{
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (!same_float(a[i].x, b[i].x) || !same_float(a[i].y, b[i].y) ||
            !same_float(a[i].z, b[i].z) || !same_float(a[i].intensity, b[i].intensity))
            return false;
    return true;
}

bool compare(Tree &tree, Point query, int count, float radius)
{
    Points reference, candidate;
    std::vector<float> reference_distance, candidate_distance;
    // Reusing these output buffers is part of the candidate contract.
    for (int repeat = 0; repeat < 2; ++repeat)
    {
        tree.Nearest_Search_Reference(query, count, reference, reference_distance, radius);
        tree.Nearest_Search(query, count, candidate, candidate_distance, radius);
        if (!same_points(reference, candidate) || reference_distance.size() != candidate_distance.size())
            return false;
        for (std::size_t i = 0; i < reference_distance.size(); ++i)
            if (!same_float(reference_distance[i], candidate_distance[i])) return false;
    }
    return true;
}

struct Timing { double milliseconds; std::size_t allocations; };

Timing benchmark(Tree &tree, const Points &queries, bool reference, int iterations)
{
    Points nearest;
    std::vector<float> distances;
    for (int i = 0; i < 20; ++i)
    {
        if (reference) tree.Nearest_Search_Reference(queries[i % queries.size()], 5, nearest, distances);
        else tree.Nearest_Search(queries[i % queries.size()], 5, nearest, distances);
    }
    allocation_count = 0;
    const auto start = std::chrono::steady_clock::now();
    track_allocations = true;
    for (int i = 0; i < iterations; ++i)
    {
        if (reference) tree.Nearest_Search_Reference(queries[i % queries.size()], 5, nearest, distances);
        else tree.Nearest_Search(queries[i % queries.size()], 5, nearest, distances);
    }
    track_allocations = false;
    const auto end = std::chrono::steady_clock::now();
    return {std::chrono::duration<double, std::milli>(end - start).count(), allocation_count};
}

int main(int argc, char **argv)
{
    const int iterations = argc > 1 ? std::atoi(argv[1]) : 20000;
    if (iterations <= 0) return 2;
    std::mt19937 generator(1337);
    std::uniform_real_distribution<float> coordinate(-10.0f, 10.0f);
    // Allocate on the heap: the upstream tree embeds a large rebuild log.
    std::unique_ptr<Tree> tree(new Tree());
    const std::vector<int> counts{1, 2, 5, 16, 96, 3, 1};
    const std::vector<float> radii{0.0f, 0.1f, 1.0f, 20.0f, INFINITY};
    std::size_t comparisons = 0;
    for (int count : counts)
    {
        if (!compare(*tree, point(0, 0, 0), count, INFINITY)) return 3;
        ++comparisons;
    }
    Points cloud;
    for (int i = 0; i < 800; ++i)
        cloud.push_back(point(coordinate(generator), coordinate(generator), coordinate(generator), float(i)));
    // Exact and nearly equal distances, duplicate coordinates with distinct IDs,
    // signed zeros and negative coordinates exercise the existing tie behavior.
    cloud.push_back(point(1, 0, 0, 801)); cloud.push_back(point(-1, 0, 0, 802));
    cloud.push_back(point(0, 1, 0, 803)); cloud.push_back(point(0, -1, 0, 804));
    cloud.push_back(point(0, 0, 1, 805)); cloud.push_back(point(0, 0, -1, 806));
    cloud.push_back(point(1, 0, 0, 807)); cloud.push_back(point(0, -0.0f, 0, 808));
    cloud.push_back(point(std::nextafter(1.0f, 2.0f), 0, 0, 809));
    tree->Build(cloud);
    Points queries;
    queries.push_back(point(0, 0, 0));
    queries.push_back(point(-0.0f, 0, 0));
    queries.push_back(point(1, 0, 0));
    queries.push_back(point(100, -100, 50));
    for (int i = 0; i < 200; ++i)
        queries.push_back(point(coordinate(generator), coordinate(generator), coordinate(generator)));
    const auto check_queries = [&]() {
        for (const Point &query : queries)
            for (int count : counts)
                for (float radius : radii)
                {
                    if (!compare(*tree, query, count, radius))
                        throw std::runtime_error("ordered neighbor/distance mismatch");
                    ++comparisons;
                }
    };
    check_queries();
    Points remove(cloud.begin(), cloud.begin() + 20);
    tree->Delete_Points(remove);
    Points added;
    for (int i = 0; i < 40; ++i)
        added.push_back(point(coordinate(generator), coordinate(generator), coordinate(generator), 900 + i));
    tree->Add_Points(added, false);
    check_queries();
    BoxPointType box{{-1.5f, -1.5f, -1.5f}, {1.5f, 1.5f, 1.5f}};
    std::vector<BoxPointType> boxes{box};
    tree->Delete_Point_Boxes(boxes);
    check_queries();
    tree->Add_Point_Boxes(boxes);
    check_queries();

    // Calls from independent workers must not share scratch or result storage.
    std::atomic<bool> parallel_ok{true};
    std::vector<std::thread> workers;
    for (int worker = 0; worker < 4; ++worker)
        workers.emplace_back([&, worker] {
            for (std::size_t i = worker; i < queries.size(); i += 4)
                for (int count : counts)
                    if (!compare(*tree, queries[i], count, INFINITY)) parallel_ok = false;
        });
    for (auto &worker : workers) worker.join();
    if (!parallel_ok) throw std::runtime_error("parallel scratch mismatch");
    comparisons += queries.size() * counts.size();

    const Timing reference1 = benchmark(*tree, queries, true, iterations);
    const Timing candidate1 = benchmark(*tree, queries, false, iterations);
    const Timing candidate2 = benchmark(*tree, queries, false, iterations);
    const Timing reference2 = benchmark(*tree, queries, true, iterations);
    if (candidate1.allocations || candidate2.allocations || !reference1.allocations)
        throw std::runtime_error("warm allocation expectation failed");
    std::cout << std::setprecision(12)
              << "RESULT_JSON {\"ordered_parity\":true,\"parallel_parity\":true,\"comparisons\":" << comparisons
              << ",\"iterations_per_trial\":" << iterations
              << ",\"reference_ms\":[" << reference1.milliseconds << ',' << reference2.milliseconds
              << "],\"candidate_ms\":[" << candidate1.milliseconds << ',' << candidate2.milliseconds
              << "],\"reference_new_allocations\":[" << reference1.allocations << ',' << reference2.allocations
              << "],\"candidate_new_allocations\":[" << candidate1.allocations << ',' << candidate2.allocations
              << "],\"scope\":\"NN kernel only; allocator counts exclude Eigen malloc and other threads\"}\n";
}
