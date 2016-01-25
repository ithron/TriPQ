#ifndef TriPQ_h
#define TriPQ_h

#include <TriPQ/StartFromLastEdge.h>
#include <TriPQ/RandomEdgeSelect.h>

#include <cassert>
#include <iterator>
#include <vector>

#if defined(TriPQLoopDetection) && !defined(NDEBUG)
#include <exception>
#include <set>
#endif

namespace TriPQ {

#if defined(TriPQLoopDetection) && defined(TriPQThrowOnLoop) && !defined(NDEBUG)
class LoopException : public std::runtime_error {
public:
  LoopException() : std::runtime_error("Found a loop."){};
};
#endif

#if defined(TriPQHopLimit) && defined(TriPQThowOnHopLimit)
class HopLimitExceeded : public std::runtime_error {
public:
  HopLimitExceeded(std::size_t n)
      : std::runtime_error("Hop count " + std::to_string(n) +
                           " exceeded limit.") {}
};
#endif

struct SingleQueryTag {};

/// Generic Point query class for 2-manifold triangulations.
///
/// This point query can be used with every 2-manifold triangulations. However
/// the decision whether a point lies on the right side of an edge of a
/// triangle must be possible and consistent.
/// Based on Brown et al. [1].
///
/// [1] P. Brown and C. T. Faigle, “A robust efficient algorithm for point
/// location in triangulations,” 1997.
///
/// \author Stefan Reinhold
///
/// \tparam Traits class conforming to the following model:
///   Types: Edge, the Edge type, best to use a handle here
///   Functors:
///     IsRightOf - binary functor, taking an edge and a point and returns true
///       if the points lies on the right hand side of the edge.
///     NextEdgeAroundOrigin - unary functor, takes an edge and returns the
///       next edge counterclockwise around the origin of the edge
///     PreviousEdgeAroundDestination - unary functor, takes an edge and returns
///       the next edge clockwise around the destination of the edge
///     OppositeEdge - unary functor, takes an edge and returns the edge
///       pointing in the opposite direction.
///
/// \tparam StartEdgePolicy a template policy class which must supply a
///   constructor taking an edge and a const member function startEdte()
///   returning an edge that should be used a starting point for the algorithm.
///   The Traits class is supplied as the template parameter.
///
/// \tparam EdgeSelectionPolicy a template policy class which must supply a
///   selectEdge(e1, e2, p) function, where e1, e2 are edged and p is a point,
///   returns either e1 or e2.
///
template <class Traits,
          template <class> class StartEdgePolicy = StartFromLastEdge,
          template <class> class EdgeSelectionPolicy = RandomEdgeSelect>
class PointQuery : private StartEdgePolicy<Traits>,
                   private EdgeSelectionPolicy<Traits> {
public:
  typedef StartEdgePolicy<Traits> StartEdge;
  typedef EdgeSelectionPolicy<Traits> SelectEdge;
  typedef typename Traits::Edge Edge;
  typedef typename Traits::EdgeDestination EdgeDestination;
  typedef typename Traits::EdgeOrigin EdgeOrigin;
  typedef typename Traits::IsRightOf IsRightOf;
  typedef typename Traits::NextEdgeAroundOrigin NextEdgeAroundOrigin;
  typedef typename Traits::OppositeEdge OppositeEdge;
  typedef typename Traits::PointsEqual PointsEqual;
  typedef typename Traits::PreviousEdgeAroundDestination
      PreviousEdgeAroundDestination;

  enum class QueryStatus {
    Success,         // Query was successfull
    HopLimitExceeded // Maximum hop count was reached, query was aborted
  };

  /// Return type of query. Can be implicitly converted to Edge.
  /// Has a status code that can be checked.
  class QueryResult {
  public:
    QueryStatus const status = QueryStatus::Success;

    QueryResult(QueryStatus s) : status(s) {}
    QueryResult(Edge e) : edge_(e) {}

    operator Edge() const { return edge_; }

    operator bool() const { return status == QueryStatus::Success; }

  private:
    Edge edge_;
  };

  PointQuery(PointQuery const &q) : PointQuery(q.startEdge()) {}
  PointQuery(PointQuery &) = default;

  /// Construct a point query foe the given triangulation
  PointQuery(Edge e0) : StartEdge(e0) {}

  /// Run a single point query
  /// \return Egde which either contains p or on which has the triangle
  /// containing p on its left side
  template <class Point>
  QueryResult operator()(Point const &p, SingleQueryTag) const {
#if defined(TriPQHopLimit)
    std::size_t hopCount = 0;
#endif
#if defined(TriPQLoopDetection) && !defined(NDEBUG)
    visitedEdges_.clear();
#endif
    auto e = this->startEdge();
    if (IsRightOf()(e, p)) e = OppositeEdge()(e);

    // According to Brown et al. [1] this loop is guaranteed to terminate
    for (;;) {
      assert(!IsRightOf()(e, p) && "p must not be right of e");

      if (PointsEqual()(p, EdgeOrigin()(e)) ||
          PointsEqual()(p, EdgeDestination()(e)))
        return e;
      // Onext in [1]
      auto const e1 = NextEdgeAroundOrigin()(e);
      // Dprev in [1]
      auto const e2 = PreviousEdgeAroundDestination()(e);
      bool const rightToE1 = IsRightOf()(e1, p);
      bool const rightToE2 = IsRightOf()(e2, p);

      if (rightToE1 && rightToE2) {
        this->foundEdge(e);
        return e;
      }
      if (!rightToE1 && rightToE2)
        e = e1;
      else if (rightToE1 && !rightToE2)
        e = e2;
      else {
        e = this->selectEdge(e1, e2, p);
      }
      this->visitEdge(e);
#if defined(TriPQHopLimit)
      ++hopCount;
#if defined(TriPQThowOnHopLimit)
      if (hopCount > TriPQHopLimit) throw HopLimitExceeded(hopCount);
#else
      if (hopCount > TriPQHopLimit)
        return QueryResult(QueryStatus::HopLimitExceeded);
#endif
#endif
    }
  }

#if defined(TriPQLoopDetection) && !defined(NDEBUG)
  void visitEdge(Edge e) const {
#ifndef TriPQThrowOnLoop
    assert(visitedEdges_.find(e) == visitedEdges_.end() && "Loop detected");
#else
    if (visitedEdges_.find(e) != visitedEdges_.end()) throw(LoopException());
#endif
    visitedEdges_.insert(e);
    StartEdge::visitEdge(e);
  }
#endif

  /// Run point queries for several points at once
  template <class InputIterator, class OutputIterator>
  void operator()(InputIterator start, InputIterator end,
                  OutputIterator out) const {
    for (auto i = start; i != end; ++i) {
      *out++ = operator()(*i, SingleQueryTag());
    }
  }

  /// Run point queries for several points at once
  template <class InputIterator>
  std::vector<Edge> operator()(InputIterator start, InputIterator end) const {
    std::vector<Edge> out(std::distance(start, end));

    operator()(start, end, out.begin());

    return out;
  }

  /// Run point queries for several points at once
  template <class Container>
  std::vector<Edge> operator()(Container const &points) const {
    return operator()(points.begin(), points.end());
  }

#if defined(TriPQLoopDetection) && !defined(NDEBUG)
private:
  mutable std::set<Edge> visitedEdges_;
#endif
};

} // namespace TriPQ

#endif // TriPQ_h
