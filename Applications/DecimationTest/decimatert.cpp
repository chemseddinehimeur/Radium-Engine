
//=============================================================================
//
//  CLASS DecimaterT - IMPLEMENTATION
//
//=============================================================================
#define OPENMESH_DECIMATER_DECIMATERT_CC

//== INCLUDES =================================================================

#include <decimatert.h>

#include <vector>
#if defined(OM_CC_MIPS)
#  include <float.h>
#else
#  include <cfloat>
#endif

//== NAMESPACE ===============================================================

//namespace OpenMesh {
//namespace Decimater {

//== IMPLEMENTATION ==========================================================

template<class Mesh>
DecimaterT<Mesh>::DecimaterT(Mesh& _mesh) :
BaseDecimaterT<Mesh>(_mesh),
    mesh_(_mesh),
#if (defined(_MSC_VER) && (_MSC_VER >= 1800)) || __cplusplus > 199711L || defined( __GXX_EXPERIMENTAL_CXX0X__ )
  heap_(nullptr)
#else
  heap_(NULL)
#endif

{

  // private vertex properties
  mesh_.add_property(collapse_target_);
  mesh_.add_property(priority_);
  mesh_.add_property(heap_position_);
}

//-----------------------------------------------------------------------------

template<class Mesh>
DecimaterT<Mesh>::~DecimaterT() {

  // private vertex properties
  mesh_.remove_property(collapse_target_);
  mesh_.remove_property(priority_);
  mesh_.remove_property(heap_position_);

}

//-----------------------------------------------------------------------------

template<class Mesh>
void DecimaterT<Mesh>::heap_vertex(VertexHandle _vh) {
  //   std::clog << "heap_vertex: " << _vh << std::endl;

  float prio, best_prio(FLT_MAX);
  typename Mesh::HalfedgeHandle heh, collapse_target;

  // find best target in one ring
  typename Mesh::VertexOHalfedgeIter voh_it(mesh_, _vh);
  for (; voh_it.is_valid(); ++voh_it) {
    heh = *voh_it;
    CollapseInfo ci(mesh_, heh);

    if (this->is_collapse_legal(ci)) {
      prio = this->collapse_priority(ci);
      if (prio >= 0.0 && prio < best_prio) {
        best_prio = prio;
        collapse_target = heh;
      }
    }
  }

  // target found -> put vertex on heap
  if (collapse_target.is_valid()) {
    //     std::clog << "  added|updated" << std::endl;
    mesh_.property(collapse_target_, _vh) = collapse_target;
    mesh_.property(priority_, _vh)        = best_prio;

    if (heap_->is_stored(_vh))
      heap_->update(_vh);
    else
      heap_->insert(_vh);
  }

  // not valid -> remove from heap
  else {
    //     std::clog << "  n/a|removed" << std::endl;
    if (heap_->is_stored(_vh))
      heap_->remove(_vh);

    mesh_.property(collapse_target_, _vh) = collapse_target;
    mesh_.property(priority_, _vh) = -1;
  }
}

//-----------------------------------------------------------------------------
template<class Mesh>
size_t DecimaterT<Mesh>::decimate(size_t _n_collapses) {

  if (!this->is_initialized())
    return 0;

  typename Mesh::VertexIter v_it, v_end(mesh_.vertices_end());
  typename Mesh::VertexHandle vp;
  typename Mesh::HalfedgeHandle v0v1;
  typename Mesh::VertexVertexIter vv_it;
  typename Mesh::VertexFaceIter vf_it;
  typedef typename Mesh::Point Point;
  unsigned int n_collapses(0);

  typedef std::vector<typename Mesh::VertexHandle> Support;
  typedef typename Support::iterator SupportIterator;

  Support support(15);
  SupportIterator s_it, s_end;

  // check _n_collapses
  if (!_n_collapses)
    _n_collapses = mesh_.n_vertices();

  // initialize heap
  HeapInterface HI(mesh_, priority_, heap_position_);

#if (defined(_MSC_VER) && (_MSC_VER >= 1800)) || __cplusplus > 199711L || defined( __GXX_EXPERIMENTAL_CXX0X__ )
  heap_ = std::unique_ptr<DeciHeap>(new DeciHeap(HI));
#else
  heap_ = std::auto_ptr<DeciHeap>(new DeciHeap(HI));
#endif


  heap_->reserve(mesh_.n_vertices());

  for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it) {
    heap_->reset_heap_position(*v_it);
    if (!mesh_.status(*v_it).deleted())
      heap_vertex(*v_it);
  }

  const bool update_normals = mesh_.has_face_normals();

  // process heap
  while ((!heap_->empty()) && (n_collapses < _n_collapses)) {
    // get 1st heap entry
    vp = heap_->front();
    v0v1 = mesh_.property(collapse_target_, vp);
    heap_->pop_front();

    // setup collapse info
    CollapseInfo ci(mesh_, v0v1);
    double x,y,z;
    getpose(&x,&y,&z,ci);

    // check topological correctness AGAIN !
    if (!this->is_collapse_legal(ci))
      continue;

    // store support (= one ring of *vp)
    vv_it = mesh_.vv_iter(ci.v0);
    support.clear();
    for (; vv_it.is_valid(); ++vv_it)
      support.push_back(*vv_it);

    Point pt(x,y,z);
    mesh_.set_point(ci.v1,pt);

    // pre-processing
    this->preprocess_collapse(ci);

    // perform collapse
    mesh_.collapse(v0v1);
    ++n_collapses;

    if (update_normals)
    {
      // update triangle normals
      vf_it = mesh_.vf_iter(ci.v1);
      for (; vf_it.is_valid(); ++vf_it)
        if (!mesh_.status(*vf_it).deleted())
          mesh_.set_normal(*vf_it, mesh_.calc_face_normal(*vf_it));
    }

    // post-process collapse
    this->postprocess_collapse(ci);

    // update heap (former one ring of decimated vertex)
    for (s_it = support.begin(), s_end = support.end(); s_it != s_end; ++s_it) {
      assert(!mesh_.status(*s_it).deleted());
      heap_vertex(*s_it);
    }

    // notify observer and stop if the observer requests it
    if (!this->notify_observer(n_collapses))
        return n_collapses;
  }

  // delete heap
  heap_.reset();



  // DON'T do garbage collection here! It's up to the application.
  return n_collapses;
}

template<class Mesh>
void DecimaterT<Mesh>::getpose(double *x, double *y, double *z,CollapseInfo _ci)
{



 Geometry::QuadricT<double> Q  =  mesh_.property(quadrics_, _ci.v0);
                            Q +=  mesh_.property(quadrics_, _ci.v1);

    using namespace Eigen;
    MatrixXd A(3,3);
    VectorXd b(3);
    VectorXd X(3);

   A(0,0) = Q.a();
   A(0,1) = Q.b();
   A(0,2) = Q.c();
   A(1,0) = Q.b();
   A(1,1) = Q.e();
   A(1,2) = Q.f();
   A(2,0) = Q.c();
   A(2,1) = Q.f();
   A(2,2) = Q.h();
   b(0)=-Q.d();
   b(1)=-Q.g();
   b(2)=-Q.i();

   X = A.colPivHouseholderQr().solve(b);
   *x=X(0);
   *y=X(1);
   *z=X(2);


}

template<typename Mesh>
void DecimaterT<Mesh>::setquad( VPropHandleT< Geometry::QuadricT<double> >  dq)
{
        quadrics_ = dq;
}

//-----------------------------------------------------------------------------

template<class Mesh>
size_t DecimaterT<Mesh>::decimate_to_faces(size_t _nv, size_t _nf) {

  if (!this->is_initialized())
    return 0;

  if (_nv >= mesh_.n_vertices() || _nf >= mesh_.n_faces())
    return 0;

  typename Mesh::VertexIter v_it, v_end(mesh_.vertices_end());
  typename Mesh::VertexHandle vp;
  typename Mesh::HalfedgeHandle v0v1;
  typename Mesh::VertexVertexIter vv_it;
  typename Mesh::VertexFaceIter vf_it;
  size_t nv = mesh_.n_vertices();
  size_t nf = mesh_.n_faces();
  unsigned int n_collapses = 0;

  typedef std::vector<typename Mesh::VertexHandle> Support;
  typedef typename Support::iterator SupportIterator;

  Support support(15);
  SupportIterator s_it, s_end;

  // initialize heap
  HeapInterface HI(mesh_, priority_, heap_position_);
  #if (defined(_MSC_VER) && (_MSC_VER >= 1800)) || __cplusplus > 199711L || defined( __GXX_EXPERIMENTAL_CXX0X__ )
    heap_ = std::unique_ptr<DeciHeap>(new DeciHeap(HI));
  #else
    heap_ = std::auto_ptr<DeciHeap>(new DeciHeap(HI));
  #endif
  heap_->reserve(mesh_.n_vertices());

  for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it) {
    heap_->reset_heap_position(*v_it);
    if (!mesh_.status(*v_it).deleted())
      heap_vertex(*v_it);
  }

  const bool update_normals = mesh_.has_face_normals();

  // process heap
  while ((!heap_->empty()) && (_nv < nv) && (_nf < nf)) {
    // get 1st heap entry
    vp = heap_->front();
    v0v1 = mesh_.property(collapse_target_, vp);
    heap_->pop_front();

    // setup collapse info
    CollapseInfo ci(mesh_, v0v1);

    // check topological correctness AGAIN !
    if (!this->is_collapse_legal(ci))
      continue;

    // store support (= one ring of *vp)
    vv_it = mesh_.vv_iter(ci.v0);
    support.clear();
    for (; vv_it.is_valid(); ++vv_it)
      support.push_back(*vv_it);

    // adjust complexity in advance (need boundary status)
    ++n_collapses;
    --nv;
    if (mesh_.is_boundary(ci.v0v1) || mesh_.is_boundary(ci.v1v0))
      --nf;
    else
      nf -= 2;

    // pre-processing
    this->preprocess_collapse(ci);

    // perform collapse
    mesh_.collapse(v0v1);

    // update triangle normals
    if (update_normals)
    {
      vf_it = mesh_.vf_iter(ci.v1);
      for (; vf_it.is_valid(); ++vf_it)
        if (!mesh_.status(*vf_it).deleted())
          mesh_.set_normal(*vf_it, mesh_.calc_face_normal(*vf_it));
    }

    // post-process collapse
    this->postprocess_collapse(ci);

    // update heap (former one ring of decimated vertex)
    for (s_it = support.begin(), s_end = support.end(); s_it != s_end; ++s_it) {
      assert(!mesh_.status(*s_it).deleted());
      heap_vertex(*s_it);
    }

    // notify observer and stop if the observer requests it
    if (!this->notify_observer(n_collapses))
        return n_collapses;
  }

  // delete heap
  heap_.reset();


  // DON'T do garbage collection here! It's up to the application.
  return n_collapses;
}

//=============================================================================
//}// END_NS_DECIMATER
//} // END_NS_OPENMESH
//=============================================================================

