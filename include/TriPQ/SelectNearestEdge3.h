#ifndef TriPQ_SelectNearestEdge3_h
#define TriPQ_SelectNearestEdge3_h

namespace TriPQ {

template <class Traits> class SelectNearestEdge3 {
public:
  typedef typename Traits::Edge Edge;
  typedef typename Traits::EdgeDestination EdgeDestination;
  typedef typename Traits::EdgeOrigin EdgeOrigin;
  typedef typename Traits::ConstructPoint ConstructPoint;

  template <class Point>
  inline Edge selectEdge(Edge e1, Edge e2, Point const &x) const {
    EdgeDestination dest;
    EdgeOrigin origin;
    ConstructPoint point;
    // e1.destination == e2.origin
    auto const o = dest(e1);
    auto const p = select ? origin(e1) : dest(e2);
    auto const op = point(p[0] - o[0], p[1] - o[1], p[2] - o[2]);
    auto const ox = point(x[0] - o[0], x[1] - o[1], x[2] - o[2]);
    auto const opox = op[0] * ox[0] + op[1] * ox[1] + op[2] * ox[2];

    select = !select;
    if (!select)
      return opox < 0 ? e2 : e1;
    else
      return opox < 0 ? e1 : e2;
  }

private:
  mutable bool select = true;
};

} // namespace TriPQ

#endif // TriPQ_SelectNearestEdge3_h
