# TriPQ
Template library for point queries in triangulations

The implemented algorithm is based on P. Brown and C. T. Faigle, “A robust
efficient algorithm for point location in triangulations,” 1997.

### Example output
```
> ./ExampleSphericalTriangulation
Triangulation dimensions:
	1000000 vertices
	1997998 triangles
	2996997 edges
Fixed starting edge, random edge select
Random points
	Querying 39999 points...Done.
	Took 9997ms
	On average 2388.45 comparisons
Sequencial points
	Querying 39999 points...Done.
	Took 8771ms
	On average 2533.23 comparisons
Fixed starting edge, nearest edge
Random points
	Querying 39999 points...Done.
	Took 6824ms
	On average 2316.96 comparisons
Sequencial points
	Querying 39999 points...Done.
	Took 6143ms
	On average 2458.96 comparisons
Last starting edge, random edge
Random points
	Querying 39999 points...Done.
	Took 9802ms
	On average 2226.8 comparisons
Sequencial points
	Querying 39999 points...Done.
	Took 71ms
	On average 23.1902 comparisons
Last starting edge, nearest edge
Random points
	Querying 39999 points...Done.
	Took 8132ms
	On average 2159.98 comparisons
Sequencial points
	Querying 39999 points...Done.
	Took 70ms
	On average 23.184 comparisons
Most located starting edge, random edge
Random points
	Querying 39999 points...Done.
	Took 23345ms
	On average 2493.2 comparisons
Sequencial points
	Querying 39999 points...Done.
	Took 20907ms
	On average 2685.08 comparisons
Most located starting edge, nearest edge
Random points
	Querying 39999 points...Done.
	Took 16851ms
	On average 2031.3 comparisons
Sequencial points
	Querying 39999 points...Done.
	Took 14647ms
	On average 2396.1 comparisons
Most located starting edge (unordered map), random edge
Random points
	Querying 39999 points...Done.
	Took 9597ms
	On average 2031.3 comparisons
Sequencial points
	Querying 39999 points...Done.
	Took 13234ms
	On average 2645.87 comparisons
Most located starting edge (unordered map), nearest edge
Random points
	Querying 39999 points...Done.
	Took 9693ms
	On average 2031.3 comparisons
Sequencial points
	Querying 39999 points...Done.
	Took 9974ms
	On average 2396.1 comparisons
```
