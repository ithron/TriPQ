# TriPQ
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
Constructing triangulation... done.
Triangulation dimensions:
	249502 vertices
	498996 triangles
	748494 edges
Fixed starting edge, random edge select
Random points
	Querying 39999 points...Done.
	Took 4265ms
	On average 1198.02 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 5783ms
	On average 1590.34 comparisons
Fixed starting edge, nearest edge
Random points
	Querying 39999 points...Done.
	Took 3510ms
	On average 1124.86 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 4754ms
	On average 1520.04 comparisons
Last starting edge, random edge
Random points
	Querying 39999 points...Done.
	Took 4778ms
	On average 1110.12 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 13ms
	On average 13.0584 comparisons
Last starting edge, nearest edge
Random points
	Querying 39999 points...Done.
	Took 3964ms
	On average 1074.88 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 12ms
	On average 13.0491 comparisons
Most located starting edge, random edge
Random points
	Querying 39999 points...Done.
	Took 11173ms
	On average 1170.32 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 11718ms
	On average 1589.25 comparisons
Most located starting edge, nearest edge
Random points
	Querying 39999 points...Done.
	Took 8907ms
	On average 1070.39 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 6652ms
	On average 1136.98 comparisons
Most located starting edge (unordered map), random edge
Random points
	Querying 39999 points...Done.
	Took 5828ms
	On average 1070.39 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 9167ms
	On average 1594 comparisons
Most located starting edge (unordered map), nearest edge
Random points
	Querying 39999 points...Done.
	Took 6054ms
	On average 1070.39 comparisons
Sequencial points
	Querying 39802 points...Done.
	Took 4379ms
	On average 1136.98 comparisons
```

## ToDo
- More documentation
- Make the example code actually readable
- TravisCI integration
- Unit Test
- More pre-defined Traits
- More examples
