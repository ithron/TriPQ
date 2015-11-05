#ifndef TriPQ_StartFromFixedEdge_h
#define TriPQ_StartFromFixedEdge_h

namespace TriPQ {

/// Naive start edge selection policy
/// Simply returns the same edge all the time
template <class Traits> class StartFromFixedEdge {
public:
  inline StartFromFixedEdge(typename Traits::Edge e) : e0_(e) {}

  inline typename Traits::Edge startEdge() const { return e0_; }

  inline void foundEdge(typename Traits::Edge) const {}

  inline void visitEdge(typename Traits::Edge) const {}

private:
  typename Traits::Edge e0_;
};

} // namespace TriPQ

#endif // TriPQ_StartFromFixedEdge_h
