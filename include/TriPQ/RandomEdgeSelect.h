#ifndef TriPQ_RandomEdgeSelect_h
#define TriPQ_RandomEdgeSelect_h

#include <random>

namespace TriPQ {

template <class Traits> class RandomEdgeSelect {
public:
  typedef typename Traits::Edge Edge;

  RandomEdgeSelect() = default;
  RandomEdgeSelect(RandomEdgeSelect &&) = default;

  template <class Point>
  inline Edge selectEdge(Edge e1, Edge e2, Point const &) const {
    return d_(gen_) ? e1 : e2;
  }

private:
  std::random_device rd_;
  mutable std::mt19937 gen_ = std::mt19937(rd_());
  mutable std::bernoulli_distribution d_ = std::bernoulli_distribution(0.5);
};

} // namespace TriPQ

#endif // TriPQ_RandomEdgeSelect_h
