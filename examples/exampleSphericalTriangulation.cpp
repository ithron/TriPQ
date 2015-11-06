#include <TriPQ/CGAL_Spherical_Polyhedron_Traits.h>
#include <TriPQ/SelectNearestEdge3.h>
#include <TriPQ/StartFromMostLocatedEdge.h>
#include <TriPQ/StartFromFixedEdge.h>
#include <TriPQ/TriPQ.h>

#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <boost/iterator/transform_iterator.hpp>
#include <CGAL/IO/Polyhedron_iostream.h>

#include <array>
#include <cmath>
#include <chrono>
#include <random>
#include <vector>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_items_with_id_3 Polyhedron_items;
typedef CGAL::Polyhedron_3<Kernel, Polyhedron_items> Polyhedron;
typedef CGAL::Triangulation_vertex_base_with_info_2<std::size_t, Kernel> Vb;
typedef CGAL::Triangulation_face_base_2<Kernel> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds> Delaunay;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Point_2 Point_2;

Point_3 sphericalToCart(Point_2 const &s) {
  using std::sin;
  using std::cos;
  return Point_3(sin(s[0]) * cos(s[1]), sin(s[0]) * sin(s[1]), -cos(s[0]));
}

inline Point_2 PhiN(Point_3 const &x) {
  return Point_2(x[0] / (1.0 - x[2]), x[1] / (1.0 - x[2]));
}

std::vector<std::array<std::size_t, 3>>
triangulate(std::vector<Point_3> &points) {
  std::size_t vertexIndex = 0;
  auto const f = [&vertexIndex](auto const &p) {
    return std::make_pair(PhiN(p), vertexIndex++);
  };

  typedef boost::transform_iterator<
      decltype(f), std::vector<Point_3>::const_iterator> Iterator;

  // Delaunay dt(Iterator(points.cbegin(), f), Iterator(points.cend(), f));
  Delaunay dt;

  // ensure that no three points inserted to dt are collinear
  std::vector<std::pair<Point_2, std::size_t>> pt(Iterator(points.cbegin(), f),
                                                  Iterator(points.cend(), f));
  auto p0 = pt[0], p1 = pt[1];
  pt.erase(pt.begin());
  pt.erase(pt.begin());
  auto iter = pt.begin();
  while (!pt.empty()) {
    auto p2 = *iter;
    if (CGAL::collinear(p0.first, p1.first, p2.first)) {
      ++iter;
      continue;
    }
    dt.push_back(iter->first)->info() = iter->second;
    p0 = p1;
    p1 = p2;
    iter = pt.erase(iter);
    if (iter == pt.end()) iter = pt.begin();
  }

  std::size_t const N = points.size();
  std::vector<std::array<std::size_t, 3>> facets;
  facets.reserve(dt.number_of_faces());
  for (auto t = dt.all_faces_begin(); t != dt.all_faces_end(); ++t) {
    auto const v0 = t->vertex(0);
    auto const v1 = t->vertex(1);
    auto const v2 = t->vertex(2);
    auto const i0 = dt.is_infinite(v0) ? N : v0->info();
    auto const i1 = dt.is_infinite(v1) ? N : v1->info();
    auto const i2 = dt.is_infinite(v2) ? N : v2->info();
    facets.push_back({i0, i1, i2});
  }
  // The infinite point is the south pole
  points.emplace_back(0, 0, 1);

  return facets;
}

std::vector<Point_3> genPoints(std::size_t const N,
                               Point_2 offset = Point_2(0, 0)) {
  std::vector<Point_3> v;
  assert(N >= 2 && "N must be at leat 2");
  v.reserve(N * (N - 2) + 2);
  for (unsigned int i = 0; i <= N; ++i) {
    double const theta = offset[0] + M_PI * double(i) / double(N);
    for (unsigned int j = 0; j < N; ++j) {
      double const phi = offset[1] + 2.0 * M_PI * double(j) / double(N);
      Point_3 const pCart(sphericalToCart(Point_2(theta, phi)));
      v.push_back(pCart);
      if (i == 0 || i == N) break;
    }
  }
  return v;
}

Polyhedron constructPolyhedron() {
  typedef Polyhedron::HalfedgeDS HDS;
  auto v = genPoints(500);
  // remove north pole
  v.pop_back();
  auto const f = triangulate(v);

  struct Builder : public CGAL::Modifier_base<HDS> {
    std::vector<Point_3> const &v_;
    std::vector<std::array<std::size_t, 3>> const &f_;
    Builder(std::vector<Point_3> const &vv,
            std::vector<std::array<std::size_t, 3>> const &ff)
        : v_(vv), f_(ff) {}
    void operator()(HDS &hds) {
      CGAL::Polyhedron_incremental_builder_3<HDS> b(hds, true);
      std::size_t vIdx = 0;
      b.begin_surface(v_.size(), f_.size());
      for (auto const &p : v_) {
        b.add_vertex(p)->id() = vIdx++;
      }
      for (auto const &t : f_) {
        b.begin_facet();
        b.add_vertex_to_facet(t[2]);
        b.add_vertex_to_facet(t[1]);
        b.add_vertex_to_facet(t[0]);
        b.end_facet();
      }
      b.end_surface();
    }
  } builder(v, f);
  Polyhedron p;
  p.delegate(builder);
  return p;
}

static std::size_t comparisonCount = 0;

template <class P>
struct CountingTraits : public TriPQ::CGALSphericalPolyhedronTraits<P> {
  typedef TriPQ::CGALSphericalPolyhedronTraits<P> Base;
  struct IsRightOf {
    template <class Point>
    inline bool operator()(typename Base::Edge e, Point const &p) const {
      ++comparisonCount;
      return typename Base::IsRightOf()(e, p);
    }
  };
};

namespace std {

template <>
struct hash<typename CountingTraits<Polyhedron>::Edge>
    : public hash<void const *> {
  size_t operator()(typename CountingTraits<Polyhedron>::Edge e) const {
    return hash<void const *>()(static_cast<void const *>(&*e));
  }
};

} // namespace std

template <class Edge, class Point>
bool assertPointInTriangle(Edge e, Point const &x) {
  typedef TriPQ::CGALSphericalPolyhedronTraits<Polyhedron> T;

  if (typename T::IsRightOf()(e, x)) return false;
  if (typename T::IsRightOf()(e->next(), x)) return false;
  if (typename T::IsRightOf()(e->next()->next(), x)) return false;
  return true;
}

template <class Query, class Points>
void runQuery(Query const &q, Points const &p) {
  comparisonCount = 0;
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  std::cout << "\tQuerying " << p.size() << " points..." << std::flush;
  TimePoint const start = Clock::now();

  auto const edges = q(p);

  using ms = std::chrono::milliseconds;
  TimePoint const end = Clock::now();
  auto const timeDiff = end - start;
  std::cout << "Done." << std::endl;
  std::cout << "\tTook " << std::chrono::duration_cast<ms>(timeDiff).count()
            << "ms" << std::endl;
  std::cout << "\tOn average " << double(comparisonCount) / double(p.size())
            << " comparisons" << std::endl;

  for (unsigned int i = 0; i < p.size(); ++i) {
    if (!assertPointInTriangle(edges[i], p[i])) {
      std::cerr << "Point not in triangle" << std::endl;
      std::cerr << "Point " << p[i] << std::endl;
      std::cerr << "Edge: (" << edges[i]->opposite()->vertex()->id() << ", "
                << edges[i]->vertex()->id() << ")" << std::endl;
      std::cerr << "Triangle " << edges[i]->vertex()->id() << ", "
                << edges[i]->next()->vertex()->id() << ", "
                << edges[i]->next()->next()->vertex()->id() << std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

int main(int, char **) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> theta(0, M_PI);
  std::uniform_real_distribution<double> phi(0, 2.0 * M_PI);

  typedef TriPQ::PointQuery<CountingTraits<Polyhedron>,
                            TriPQ::StartFromFixedEdge,
                            TriPQ::RandomEdgeSelect> QueryFixedRandom;
  typedef TriPQ::PointQuery<CountingTraits<Polyhedron>,
                            TriPQ::StartFromFixedEdge,
                            TriPQ::SelectNearestEdge3> QueryFixedNearest;
  typedef TriPQ::PointQuery<CountingTraits<Polyhedron>,
                            TriPQ::StartFromLastEdge,
                            TriPQ::RandomEdgeSelect> QueryLastRandom;
  typedef TriPQ::PointQuery<CountingTraits<Polyhedron>,
                            TriPQ::StartFromLastEdge,
                            TriPQ::SelectNearestEdge3> QueryLastNearest;
  typedef TriPQ::PointQuery<CountingTraits<Polyhedron>,
                            TriPQ::StartFromMostLocatedEdge,
                            TriPQ::RandomEdgeSelect> QueryMRRandom;
  typedef TriPQ::PointQuery<CountingTraits<Polyhedron>,
                            TriPQ::StartFromMostLocatedEdge,
                            TriPQ::SelectNearestEdge3> QueryMRNearest;
  typedef TriPQ::PointQuery<CountingTraits<Polyhedron>,
                            TriPQ::StartFromMostLocatedEdgeUnordered,
                            TriPQ::RandomEdgeSelect> QueryMRURandom;
  typedef TriPQ::PointQuery<CountingTraits<Polyhedron>,
                            TriPQ::StartFromMostLocatedEdgeUnordered,
                            TriPQ::SelectNearestEdge3> QueryMRUNearest;

  std::size_t const N = 40000;
  // std::size_t const N = 4;

  // Generate polyhedron
  std::cout << "Constructing triangulation... " << std::flush;
  auto const p = constructPolyhedron();
  std::cout << "done." << std::endl;

  std::vector<Point_3> randomPoints(N - 1);
  // generate query points
  for (unsigned int i = 0; i < N - 1; ++i) {
    randomPoints[i] = sphericalToCart(Point_2(theta(gen), phi(gen)));
  }

  auto const sequencialPoints =
      genPoints(std::sqrt(N), Point_2(theta(gen), phi(gen)));

  auto const e0 = p.halfedges_begin();

  std::cout << "Triangulation dimensions:" << std::endl;
  std::cout << "\t" << p.size_of_vertices() << " vertices" << std::endl;
  std::cout << "\t" << p.size_of_facets() << " triangles" << std::endl;
  std::cout << "\t" << p.size_of_halfedges() / 2 << " edges" << std::endl;

  std::cout << "Fixed starting edge, random edge select" << std::endl;
  std::cout << "Random points" << std::endl;
  runQuery(QueryFixedRandom(e0), randomPoints);
  std::cout << "Sequencial points" << std::endl;
  runQuery(QueryFixedRandom(e0), sequencialPoints);

  std::cout << "Fixed starting edge, nearest edge" << std::endl;
  std::cout << "Random points" << std::endl;
  runQuery(QueryFixedNearest(e0), randomPoints);
  std::cout << "Sequencial points" << std::endl;
  runQuery(QueryFixedNearest(e0), sequencialPoints);

  std::cout << "Last starting edge, random edge" << std::endl;
  std::cout << "Random points" << std::endl;
  runQuery(QueryLastRandom(e0), randomPoints);
  std::cout << "Sequencial points" << std::endl;
  runQuery(QueryLastRandom(e0), sequencialPoints);

  std::cout << "Last starting edge, nearest edge" << std::endl;
  std::cout << "Random points" << std::endl;
  runQuery(QueryLastNearest(e0), randomPoints);
  std::cout << "Sequencial points" << std::endl;
  runQuery(QueryLastNearest(e0), sequencialPoints);

  std::cout << "Most located starting edge, random edge" << std::endl;
  std::cout << "Random points" << std::endl;
  runQuery(QueryMRRandom(e0), randomPoints);
  std::cout << "Sequencial points" << std::endl;
  runQuery(QueryMRRandom(e0), sequencialPoints);

  std::cout << "Most located starting edge, nearest edge" << std::endl;
  std::cout << "Random points" << std::endl;
  runQuery(QueryMRNearest(e0), randomPoints);
  std::cout << "Sequencial points" << std::endl;
  runQuery(QueryMRNearest(e0), sequencialPoints);

  std::cout << "Most located starting edge (unordered map), random edge"
            << std::endl;
  std::cout << "Random points" << std::endl;
  runQuery(QueryMRUNearest(e0), randomPoints);
  std::cout << "Sequencial points" << std::endl;
  runQuery(QueryMRURandom(e0), sequencialPoints);

  std::cout << "Most located starting edge (unordered map), nearest edge"
            << std::endl;
  std::cout << "Random points" << std::endl;
  runQuery(QueryMRUNearest(e0), randomPoints);
  std::cout << "Sequencial points" << std::endl;
  runQuery(QueryMRUNearest(e0), sequencialPoints);

  return EXIT_SUCCESS;
}
