# Nearest-search allocation refactor validation

Run with the existing C++/PCL development toolchain:

```sh
python3 tests/nearest_search_parity.py --output nearest_search_result.json
```

The script generates a temporary test-only declaration and implementation of
the original nearest-search method. Production headers and binaries contain no
reference-method hooks. Both methods use the same tree, traversal, heap
comparator and synchronization. The original method fixture is from commit
`e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.

The test uses FAST-LIO's aligned `pcl::PointXYZINormal` type and compares ordered
point coordinates/intensity/normal/curvature and distances bit for bit
for empty trees, signed zero, negative/random coordinates, duplicate points,
distance ties, radius cutoffs, varying K, point/box deletion and re-addition, and
independent parallel queries. Map mutations remain below the background rebuild
threshold for deterministic comparison. This does not demonstrate estimator
equivalence during a concurrently changing tree; complete estimator replay and
baseline repeatability checks are required separately.

Alternating reference/candidate kernel timing trials are included. The warm-loop
allocation counter counts C++ `new`/`new[]` calls on the query thread; it excludes
Eigen's direct aligned `malloc` and allocations on other threads. These kernel
timings are not whole-process or flight CPU measurements.

The candidate retains one scratch heap per point type and calling thread, sized
to the largest requested K on that thread. Output-vector capacity remains owned
by the caller. FAST-LIO additionally retains its small distance vector per
matching worker. No neighbor-selection, tie-order, maximum-distance, tree
rebuild or estimator-quality setting is changed.
