#include <Core/Animation/PoseOperation.hpp>
#include <Eigen/Geometry>

namespace Ra {
namespace Core {
namespace Animation {

bool compatible( const Pose& p0, const Pose& p1 ) {
    return ( p0.size() == p1.size() );
}

Pose relativePose( const Pose& modelPose, const RestPose& restPose ) {
    CORE_ASSERT( compatible( modelPose, restPose ), " Poses with different size " );
    Pose T( restPose.size() );
#pragma omp parallel for
    for ( int i = 0; i < int( T.size() ); ++i )
    {
        T[i] = modelPose[i] * restPose[i].inverse( Eigen::Affine );
    }
    return T;
}

Pose applyTransformation( const Pose& pose, const AlignedStdVector<Transform>& transform ) {
    Pose T( std::min( pose.size(), transform.size() ) );
#pragma omp parallel for
    for ( int i = 0; i < int( T.size() ); ++i )
    {
        T[i] = transform[i] * pose[i];
    }
    return T;
}

Pose applyTransformation( const Pose& pose, const Transform& transform ) {
    Pose T( pose.size() );
#pragma omp parallel for
    for ( int i = 0; i < int( T.size() ); ++i )
    {
        T[i] = transform * pose[i];
    }
    return T;
}

bool areEqual( const Pose& p0, const Pose& p1 ) {
    CORE_ASSERT( compatible( p0, p1 ), " Poses with different size " );
    const uint n = p0.size();
    for ( uint i = 0; i < n; ++i )
    {
        if ( !p0[i].isApprox( p1[i] ) )
        {
            return false;
        }
    }
    return true;
}

Pose interpolatePoses( const Pose& a, const Pose& b, const Scalar t ) {
    CORE_ASSERT( ( a.size() == b.size() ), "Poses are wrong" );
    CORE_ASSERT( ( ( t >= Scalar( 0. ) ) && ( t <= Scalar( 1. ) ) ), "T is wrong" );

    const uint size = a.size();
    Pose interpolatedPose( size );

#pragma omp parallel for
    for ( int i = 0; i < int( size ); ++i )
    {
        interpolateTransforms( a[i], b[i], t, interpolatedPose[i] );
    }

    return interpolatedPose;
}

void interpolateTransforms( const Ra::Core::Transform& a, const Ra::Core::Transform& b,
                            const Scalar t, Ra::Core::Transform& interpolated ) {
    Ra::Core::Matrix3 aR, bR;
    Ra::Core::Matrix3 aS, bS;
    a.computeRotationScaling( &aR, &aS );
    b.computeRotationScaling( &bR, &bS );

    Ra::Core::Quaternion aRot = Ra::Core::Quaternion( aR );
    Ra::Core::Quaternion bRot = Ra::Core::Quaternion( bR );
    Ra::Core::Quaternion iR = aRot.slerp( t, bRot );
    Ra::Core::Matrix3 iS = ( 1 - t ) * aS + t * bS;
    Ra::Core::Vector3 iT = ( 1 - t ) * a.translation() + t * b.translation();

    interpolated.fromPositionOrientationScale( iT, iR, iS.diagonal() );
}

} // namespace Animation
} // namespace Core
} // namespace Ra
