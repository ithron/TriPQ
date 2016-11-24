#ifndef TriPQ_CGAL_Spherical_Polyhedron_Traits_h
#define TriPQ_CGAL_Spherical_Polyhedron_Traits_h

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

namespace TriPQ {

/// Assumes counterclockwise orientation of edges inside a triangle
template <class Polyhedron, class Derived>
struct CGALSphericalPolyhedronTraitsBase {
  typedef typename Polyhedron::Halfedge_const_handle Edge;
  typedef typename Polyhedron::Vertex_const_handle Vertex;
  typedef typename Polyhedron::Point_3 Point;

  static double epsilon() { return 1e-12; }

  struct NextEdgeAroundOrigin {
    inline Edge operator()(Edge e) const { return e->prev()->opposite(); }
  };

  struct PreviousEdgeAroundDestination {
    inline Edge operator()(Edge e) const { return e->next()->opposite(); }
  };

  struct NextEdge {
    inline Edge operator()(Edge e) const { return e->next(); }
  };

  struct OppositeEdge {
    inline Edge operator()(Edge e) const { return e->opposite(); }
  };

  struct VertexPoint {
    inline auto const &operator()(Vertex v) const { return v->point(); }
  };

  struct ConstructPoint {
    template <class RT>
    inline decltype(auto) operator()(RT x, RT y, RT z) const {
      return Point(x, y, z);
    }
  };

  struct EdgeOriginVertex {
    inline decltype(auto) operator()(Edge e) const {
      return e->opposite()->vertex();
    }
  };

  struct EdgeDestinationVertex {
    inline decltype(auto) operator()(Edge e) const { return e->vertex(); }
  };

  struct EdgeOrigin {
    inline auto operator()(Edge e) const {
      return typename Derived::VertexPoint()(
          typename Derived::EdgeOriginVertex()(e));
    }
  };

  struct EdgeDestination {
    inline auto operator()(Edge e) const {
      return typename Derived::VertexPoint()(
          typename Derived::EdgeDestinationVertex()(e));
    }
  };

  struct IsRightOf {
    template <class Pt> inline bool operator()(Edge e, Pt const &p) const {
      typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
      typedef K::Vector_3 Vector;
      auto const a = typename Derived::EdgeOrigin()(e);
      auto const b = typename Derived::EdgeDestination()(e);
      auto const c =
          typename Derived::EdgeDestination()(typename Derived::NextEdge()(e));

      auto const testOrientation =
          CGAL::orientation(Vector(p[0], p[1], p[2]), Vector(a[0], a[1], a[2]),
                            Vector(b[0], b[1], b[2]));

      if (testOrientation == CGAL::COPLANAR) return false;

      auto const refOrientation =
          CGAL::orientation(Vector(a[0], a[1], a[2]), Vector(b[0], b[1], b[2]),
                            Vector(c[0], c[1], c[2]));

      return testOrientation != refOrientation;
    }
  };

  struct PointsEqual {
    template <class Pt1, class Pt2> inline bool operator()(Pt1 p1, Pt2 p2) const {
      typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
      typedef K::Vector_3 Vector;
      Vector const a(p1[0], p1[1], p1[2]);
      Vector const b(p2[0], p2[1], p2[2]);
      auto const dist = (a - b).squared_length();
      return dist < Derived::epsilon();
    }
  };
};

template <class Polyhedron>
struct CGALSphericalPolyhedronTraits
    : CGALSphericalPolyhedronTraitsBase<
          Polyhedron, CGALSphericalPolyhedronTraits<Polyhedron>> {};

} // namespace TriPQ

#endif // TriPQ_CGAL_Spherical_Polyhedron_Traits_h
