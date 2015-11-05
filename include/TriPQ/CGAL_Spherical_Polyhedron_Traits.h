#ifndef TriPQ_CGAL_Spherical_Polyhedron_Traits_h
#define TriPQ_CGAL_Spherical_Polyhedron_Traits_h

namespace TriPQ {

template <class Polyhedron> struct CGALSphericalPolyhedronTraits {
  typedef typename Polyhedron::Edge_const_handle Edge;
  typedef typename Polyhedron::Vertex_const_handle Vertex;

  struct NextEdge {
    inline Edge operator()(Edge e) const { return e->next(); }
  };

  struct OppositeEdge {
    inline Edge operator()(Edge e) const { return e->opposite(); }
  };

  struct VertexPoint {
    inline decltype(auto) operator()(Vertex v) const { return v->point(); }
  };

  struct ConstructPoint {
    template <class RT>
    inline decltype(auto) operator()(RT x, RT y, RT z) const {
      return typename Polyhedron::Point_3(x, y, z);
    }
  };

  struct EdgeOrigin {
    inline decltype(auto) operator()(Edge e) const {
      return VertexPoint()(e->opposite()->vertex());
    }
  };

  struct EdgeDestination {
    inline decltype(auto) operator()(Edge e) const {
      return VertexPoint()(e->vertex());
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
    template <class Point>
    inline decltype(auto) operator()(Edge e, Point const &p) const {
      return Determinant()(EdgeOrigin(e), EdgeDestination(e), p) < 0;
    }
  };
};

} // namespace TriPQ

#endif // TriPQ_CGAL_Spherical_Polyhedron_Traits_h
