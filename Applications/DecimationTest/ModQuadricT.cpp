

#define OPENMESH_DECIMATER_MODQUADRIC_CC

//== INCLUDES =================================================================

#include <ModQuadricT.h>


//== NAMESPACE ===============================================================

//namespace OpenMesh { // BEGIN_NS_OPENMESH
//namespace Decimater { // BEGIN_NS_DECIMATER


//== IMPLEMENTATION ==========================================================


template<class DecimaterType>
void
ModQuadricT<DecimaterType>::
initialize()
{
  using Geometry::Quadricd;
  // alloc quadrics
  if (!quadrics_.is_valid())
    Base::mesh().add_property( quadrics_ );

  // clear quadrics
  typename Mesh::VertexIter  v_it  = Base::mesh().vertices_begin(),
                             v_end = Base::mesh().vertices_end();

  for (; v_it != v_end; ++v_it)
    Base::mesh().property(quadrics_, *v_it).clear();

  // calc (normal weighted) quadric
  typename Mesh::FaceIter          f_it  = Base::mesh().faces_begin(),
                                   f_end = Base::mesh().faces_end();

  typename Mesh::FaceVertexIter    fv_it;
  typename Mesh::VertexHandle      vh0, vh1, vh2;
  typedef Vec3d                    Vec3;

  for (; f_it != f_end; ++f_it)
  {
    fv_it = Base::mesh().fv_iter(*f_it);
    vh0 = *fv_it;  ++fv_it;
    vh1 = *fv_it;  ++fv_it;
    vh2 = *fv_it;

    Vec3 v0, v1, v2;
    {
      using namespace OpenMesh;

      v0 = vector_cast<Vec3>(Base::mesh().point(vh0));
      v1 = vector_cast<Vec3>(Base::mesh().point(vh1));
      v2 = vector_cast<Vec3>(Base::mesh().point(vh2));
    }

    Vec3 n = (v1-v0) % (v2-v0);
    double area = n.norm();
    if (area > FLT_MIN)
    {
      n /= area;
      area *= 0.5;
    }

    const double a = n[0];
    const double b = n[1];
    const double c = n[2];
    const double d = -(vector_cast<Vec3>(Base::mesh().point(vh0))|n);

    Quadricd q(a, b, c, d);
    q *= area;

    Base::mesh().property(quadrics_, vh0) += q;
    Base::mesh().property(quadrics_, vh1) += q;
    Base::mesh().property(quadrics_, vh2) += q;
  }
}

//-----------------------------------------------------------------------------

template<class MeshT>
void ModQuadricT<MeshT>::set_error_tolerance_factor(double _factor) {
  if (this->is_binary()) {
    if (_factor >= 0.0 && _factor <= 1.0) {
      // the smaller the factor, the smaller max_err_ gets
      // thus creating a stricter constraint
      // division by error_tolerance_factor_ is for normalization
      double max_err = max_err_ * _factor / this->error_tolerance_factor_;
      set_max_err(max_err);
      this->error_tolerance_factor_ = _factor;

      initialize();
    }
  }
}

//=============================================================================
//} // END_NS_DECIMATER
//} // END_NS_OPENMESH
//=============================================================================
