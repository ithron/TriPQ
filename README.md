# TriPQ [![Build Status](https://travis-ci.org/ithron/TriPQ.svg?branch=master)](https://travis-ci.org/ithron/TriPQ)
Template library for point queries in triangulations

The implemented algorithm is based on P. Brown and C. T. Faigle, “A robust
efficient algorithm for point location in triangulations,” 1997.

## Build Instructions
TriPQ is a header only library, so copying `include/TriPQ` to your project's source folder is totally fine.
However it is recommended to use `cmake` to do the build-configuration:
```
git clone git@github.com:ithron/TriPQ.git
mkdir TriPQ-build && cd TriPQ-build
cmake ../TriPQ
make && sudo make install
```
This will install the headers into e.g. `/usr/local/include/TriPQ` and also installs a cmake configuration file into `/usr/local/lib/cmake/TriPQ`.
When using cmake for your project configuration just set `TriPQ_DIR` to `/usr/lib/cmake/TriPQ` and write in your CMakeLists.txt
```
find_package(TriPQ)
...
target_link_libraries(MyTarget .... TriPQ)
```
Be aware that TriPQ uses c++14 features, so using a modern compiler (e.g. clang-3.5 or gcc-5) is required.

### Notes
The termination of the algorithm with the nearest edge selection policy is poofed in [1]. However, this only holds for planar triangulations. For example on spherical 2-manifold meshes the algorithm is not guaranteed to terminate when using the nearest edge policy (it does for delaunay triangulations of course). For that cases the random edge selection policy is a better choice.

Also note that in the given example a uniform random distribution of test points is used. Therefore the most located edge strategy does not perform better than the other strageties. This should be different for a biased distribution, like a gaussian.


### Example output
```
> ./ExampleSphericalTriangulation
Triangulation dimensions:
	249502 vertices
	498996 triangles
	748494 edges
Fixed starting edge, random edge select
Random points
	Querying 39999 points...Done.
	Took 5800ms
	On average 1195.63 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 7012ms
	On average 1502.26 comparisons
Fixed starting edge, nearest edge
Random points
	Querying 39999 points...Done.
	Took 5153ms
	On average 1122.14 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 6444ms
	On average 1431.89 comparisons
Last starting edge, random edge
Random points
	Querying 39999 points...Done.
	Took 5560ms
	On average 1099.87 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 20ms
	On average 13.0618 comparisons
Last starting edge, nearest edge
Random points
	Querying 39999 points...Done.
	Took 4889ms
	On average 1065.02 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 20ms
	On average 13.0603 comparisons
Most located starting edge, random edge
Random points
	Querying 39999 points...Done.
	Took 13149ms
	On average 1169.87 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 12964ms
	On average 1506.1 comparisons
Most located starting edge, nearest edge
Random points
	Querying 39999 points...Done.
	Took 10818ms
	On average 1067.99 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 7628ms
	On average 1001.63 comparisons
Most located starting edge (unordered map), random edge
Random points
	Querying 39999 points...Done.
	Took 7687ms
	On average 1067.99 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 7281ms
	On average 1115.46 comparisons
Most located starting edge (unordered map), nearest edge
Random points
	Querying 39999 points...Done.
	Took 7619ms
	On average 1067.99 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 6104ms
	On average 1001.63 comparisons
```

## ToDo
- More documentation
- Make the example code actually readable
- Unit Test
- More pre-defined Traits
- More examples
