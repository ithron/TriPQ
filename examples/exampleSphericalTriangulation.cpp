#include <TriPQ/TriPQ.h>
#include <TriPQ/CGAL_Spherical_Polyhedron_Traits.h>

#include <CGAL/Triangulation_vertex_base_with_info_2.h>
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
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
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

  Delaunay dt(Iterator(points.cbegin(), f), Iterator(points.cend(), f));

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
  // The infinite point is the north pole
  points.emplace_back(0, 0, 1);

  return facets;
}

std::vector<Point_3> genPoints() {
  std::vector<Point_3> v;
  unsigned int N = 100;
  unsigned int M = 100;
  v.reserve(N * M);
  for (unsigned int i = 0; i < N; ++i) {
    double const theta = M_PI * double(i) / double(N);
    for (unsigned int j = 0; j < M; ++j) {
      double const phi = 2.0 * M_PI * double(j) / double(M);
      v.push_back(sphericalToCart(Point_2(theta, phi)));
    }
  }
  // remove the north pole
  v.pop_back();
  return v;
}

Polyhedron constructPolyhedron() {
  typedef Polyhedron::HalfedgeDS HDS;
  auto v = genPoints();
  auto const f = triangulate(v);

  struct Builder : public CGAL::Modifier_base<HDS> {
    std::vector<Point_3> const &v_;
    std::vector<std::array<std::size_t, 3>> const &f_;
    Builder(std::vector<Point_3> const &vv,
            std::vector<std::array<std::size_t, 3>> const &ff)
        : v_(vv), f_(ff) {}
    void operator()(HDS &hds) {
      CGAL::Polyhedron_incremental_builder_3<HDS> b(hds, true);
      b.begin_surface(v_.size(), f_.size());
      for (auto const &p : v_) {
        b.add_vertex(p);
      }
      for (auto const &t : f_) {
        b.begin_facet();
        b.add_vertex_to_facet(t[0]);
        b.add_vertex_to_facet(t[1]);
        b.add_vertex_to_facet(t[2]);
        b.end_facet();
      }
      b.end_surface();
    }
  } builder(v, f);
  Polyhedron p;
  p.delegate(builder);
  return p;
}

int main(int, char **) {
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> theta(0, M_PI);
  std::uniform_real_distribution<double> phi(0, 2.0 * M_PI);

  typedef TriPQ::PointQuery<TriPQ::CGALSphericalPolyhedronTraits<Polyhedron>>
      Query;

  std::size_t const N = 1000000;

  // Generate polyhedron
  auto const p = constructPolyhedron();

  std::vector<Point_3> queryPoints(N);
  // generate query points
  for (unsigned int i = 0; i < N; ++i) {
    queryPoints[i] = sphericalToCart(Point_2(theta(gen), phi(gen)));
  }

  std::cout << "Querying " << queryPoints.size() << " points..." << std::flush;
  TimePoint const start = Clock::now();

  Query q(p.halfedges_begin());
  q(queryPoints);

  using ms = std::chrono::milliseconds;
  TimePoint const end = Clock::now();
  auto const timeDiff = end - start;
  std::cout << "Done." << std::endl;
  std::cout << "Took " << std::chrono::duration_cast<ms>(timeDiff).count()
            << "ms" << std::endl;

  return EXIT_SUCCESS;
}
