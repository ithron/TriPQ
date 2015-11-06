# TriPQ
Template library for point queries in triangulations

The implemented algorithm is based on P. Brown and C. T. Faigle, “A robust
efficient algorithm for point location in triangulations,” 1997.

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
