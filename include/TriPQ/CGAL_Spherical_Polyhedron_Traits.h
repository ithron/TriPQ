#ifndef TriPQ_CGAL_Spherical_Polyhedron_Traits_h
#define TriPQ_CGAL_Spherical_Polyhedron_Traits_h

namespace TriPQ {

/// Assumes counterclockwise orientation of edges inside a triangle
template <class Polyhedron> struct CGALSphericalPolyhedronTraits {
  typedef typename Polyhedron::Halfedge_const_handle Edge;
  typedef typename Polyhedron::Vertex_const_handle Vertex;
  typedef typename Polyhedron::Point_3 Point;

  struct NextEdgeAroundOrigin {
    inline Edge operator()(Edge e) const {
      auto const v = EdgeOriginVertex()(e);
      auto const eOp = OppositeEdge()(e);
      auto ei = v->vertex_begin();
      while (Edge(&*ei) != eOp) ++ei;
      return OppositeEdge()(Edge(&*--ei));
    }
  };

  struct PreviousEdgeAroundDestination {
    inline Edge operator()(Edge e) const {
      auto const v = EdgeDestinationVertex()(e);
      auto ei = v->vertex_begin();
      while (Edge(&*ei) != e) ++ei;
      return Edge(&*++ei);
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
      return typename Polyhedron::Point_3(x, y, z);
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
      return VertexPoint()(EdgeOriginVertex()(e));
    }
  };

  struct EdgeDestination {
    inline Point const &operator()(Edge e) const {
      return VertexPoint()(EdgeDestinationVertex()(e));
    }
  };

  struct Determinant {
    template <class A, class B, class C>
    inline decltype(auto) operator()(A const &a, B const &b, C const &c) const {
      return a[0] * (b[1] * c[2] - b[2] * c[1]) -
             b[0] * (a[1] * c[2] - a[2] * c[1]) +
             c[0] * (a[1] * b[2] - a[2] * b[1]);
    }
  };

  struct IsRightOf {
    template <class Pt> inline bool operator()(Edge e, Pt const &p) const {
      return Determinant()(p, EdgeOrigin()(e), EdgeDestination()(e)) < 0;
    }
  };
};

} // namespace TriPQ

#endif // TriPQ_CGAL_Spherical_Polyhedron_Traits_h
