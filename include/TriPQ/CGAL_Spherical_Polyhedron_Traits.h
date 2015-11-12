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

  struct NextEdgeAroundOrigin {
    inline Edge operator()(Edge e) const {
      // auto const v = EdgeOriginVertex()(e);
      // auto const eOp = OppositeEdge()(e);
      // auto ei = v->vertex_begin();
      // while (Edge(&*ei) != eOp) ++ei;
      // return OppositeEdge()(Edge(&*--ei));
      return e->prev()->opposite();
    }
  };

  struct PreviousEdgeAroundDestination {
    inline Edge operator()(Edge e) const {
      // auto const v = EdgeDestinationVertex()(e);
      // auto ei = v->vertex_begin();
      // while (Edge(&*ei) != e) ++ei;
      // return Edge(&*++ei);
      return e->next()->opposite();
    }
  };

  struct OppositeEdge {
    inline Edge operator()(Edge e) const { return e->opposite(); }
  };

  struct VertexPoint {
    inline Point const &operator()(Vertex v) const { return v->point(); }
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
    inline Point const &operator()(Edge e) const {
      return typename Derived::VertexPoint()(
          typename Derived::EdgeOriginVertex()(e));
    }
  };

  struct EdgeDestination {
    inline Point const &operator()(Edge e) const {
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

      return CGAL::orientation(Vector(p[0], p[1], p[2]),
                               Vector(a[0], a[1], a[2]),
                               Vector(b[0], b[1], b[2])) == CGAL::NEGATIVE;
    }
  };
};

template <class Polyhedron>
struct CGALSphericalPolyhedronTraits
    : CGALSphericalPolyhedronTraitsBase<
          Polyhedron, CGALSphericalPolyhedronTraits<Polyhedron>> {};

} // namespace TriPQ

#endif // TriPQ_CGAL_Spherical_Polyhedron_Traits_h
