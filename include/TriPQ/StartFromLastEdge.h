#ifndef TriPQ_StartFromLastEdge_h
#define TriPQ_StartFromLastEdge_h

namespace TriPQ {

/// Start edge selection policy that returns the result of the last query as
/// the start edge for the next query
template <class Traits> class StartFromLastEdge {
public:
  inline StartFromLastEdge(typename Traits::Edge e) : eLast_(e) {}

  inline typename Traits::Edge startEdge() const { return eLast_; }

  inline void foundEdge(typename Traits::Edge e) const { eLast_ = e; }

  inline void visitEdge(typename Traits::Edge) const {}

private:
  mutable typename Traits::Edge eLast_;
};

} // namespace TriPQ

#endif // TriPQ_StartFromLastEdge_h
