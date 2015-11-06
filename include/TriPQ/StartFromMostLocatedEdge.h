#ifndef TriPQ_StartFromMostLocatedEdge_h
#define TriPQ_StartFromMostLocatedEdge_h

#include <map>
#include <unordered_map>
#include <utility>

namespace TriPQ {

template <class Traits, class Map> class StartFromMostLocatedEdgeBase {
  typedef typename Traits::Edge Edge;

public:
  inline StartFromMostLocatedEdgeBase(Edge e) : mostLocatedEdge_(e, 0) {}

  inline Edge startEdge() const { return mostLocatedEdge_.first; }

  inline void foundEdge(Edge) const {}

  inline void visitEdge(Edge e) const {
    auto search = edgeCounts_.find(e);
    if (search == edgeCounts_.end()) {
      // new edge
      search = edgeCounts_.emplace(e, 0).first;
    }
    ++search->second;
    if (search->second > mostLocatedEdge_.second) {
      mostLocatedEdge_ = *search;
    }
  }

private:
  mutable std::pair<Edge, std::size_t> mostLocatedEdge_;
  mutable Map edgeCounts_;
};

template <class Traits>
using StartFromMostLocatedEdge =
    StartFromMostLocatedEdgeBase<Traits,
                                 std::map<typename Traits::Edge, std::size_t>>;

template <class Traits>
using StartFromMostLocatedEdgeUnordered = StartFromMostLocatedEdgeBase<
    Traits, std::unordered_map<typename Traits::Edge, std::size_t>>;

} // namespace TriPQ

#endif // TriPQ_StartFromMostLocatedEdge_h
