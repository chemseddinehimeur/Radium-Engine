#include <Core/RaCore.hpp>

#include <Core/Mesh/TopologicalTriMesh/TopologicalMesh.hpp>
#include <unordered_map>

#include <Core/Log/Log.hpp>

namespace Ra
{
namespace Core
{

template <typename T>
using PropPair = std::pair<AttribHandle<T>, OpenMesh::HPropHandleT<T>>;

template <typename T>
void addAttribPairToTopo( const TriangleMesh& triMesh,
                          TopologicalMesh* topoMesh,
                          AttribManager::value_type attr,
                          std::vector<PropPair<T>>& vprop,
                          std::vector<OpenMesh::HPropHandleT<T>>& pph )
{
    AttribHandle<T> h = triMesh.attribManager().getAttribHandle<T>( attr->getName() );
    OpenMesh::HPropHandleT<T> oh;
    topoMesh->add_property( oh, attr->getName() );
    vprop.push_back( std::make_pair( h, oh ) );
    pph.push_back( oh );
}

template <typename T>
void addAttribPairToCore( TriangleMesh& triMesh,
                          TopologicalMesh* topoMesh,
                          OpenMesh::HPropHandleT<T> oh,
                          std::vector<PropPair<T>>& vprop )
{
    AttribHandle<T> h{triMesh.attribManager().addAttrib<T>( topoMesh->property( oh ).name() )};
    vprop.push_back( std::make_pair( h, oh ) );
}

template <typename T>
void copyAttribToTopo( const TriangleMesh& triMesh,
                       TopologicalMesh* topoMesh,
                       std::vector<PropPair<T>>& vprop,
                       TopologicalMesh::HalfedgeHandle heh,
                       unsigned int vindex )
{
    for ( auto pp : vprop )
    {
        topoMesh->property( pp.second, heh ) =
            triMesh.attribManager().getAttrib( pp.first ).data()[vindex];
    }
}

template <typename T>
void copyAttribToCore( TriangleMesh& triMesh,
                       TopologicalMesh* topoMesh,
                       std::vector<PropPair<T>>& vprop,
                       TopologicalMesh::HalfedgeHandle heh )
{
    for ( auto pp : vprop )
    {
        triMesh.attribManager()
            .getAttrib( pp.first )
            .data()
            .push_back( topoMesh->property( pp.second, heh ) );
    }
}

TopologicalMesh::TopologicalMesh( const TriangleMesh& triMesh )
{
    struct hash_vec
    {
        size_t operator()( const Vector3& lvalue ) const
        {
            return lvalue[0] + lvalue[1] + lvalue[2] + floor( lvalue[0] ) * 1000.f +
                   floor( lvalue[1] ) * 1000.f + floor( lvalue[2] ) * 1000.f;
        }
    };

    using vMap = std::unordered_map<Vector3, TopologicalMesh::VertexHandle, hash_vec>;

    vMap vertexHandles;
    std::vector<PropPair<float>> vprop_float;
    std::vector<PropPair<Vector2>> vprop_vec2;
    std::vector<PropPair<Vector3>> vprop_vec3;
    std::vector<PropPair<Vector4>> vprop_vec4;

    // loop over all attribs and build correspondance pair
    for ( auto attr : triMesh.attribManager().attribs() )
    {
        // skip builtin attribs
        if ( attr->getName() != std::string( "in_position" ) &&
             attr->getName() != std::string( "in_normal" ) )
        {
            LOG( logINFO ) << "to TOPO found attrib " << attr->getName() << "\n;";
            if ( attr->isFloat() )
                addAttribPairToTopo( triMesh, this, attr, vprop_float, m_floatPph );
            if ( attr->isVec2() ) addAttribPairToTopo( triMesh, this, attr, vprop_vec2, m_vec2Pph );
            if ( attr->isVec3() ) addAttribPairToTopo( triMesh, this, attr, vprop_vec3, m_vec3Pph );
            if ( attr->isVec4() ) addAttribPairToTopo( triMesh, this, attr, vprop_vec4, m_vec4Pph );
        }
    }

    uint num_triangles = triMesh.m_triangles.size();

    for ( unsigned int i = 0; i < num_triangles; i++ )
    {
        std::vector<TopologicalMesh::VertexHandle> face_vhandles;
        std::vector<TopologicalMesh::Normal> face_normals;
        std::vector<unsigned int> face_vertexIndex;

        for ( int j = 0; j < 3; ++j )
        {
            unsigned int inMeshVertexIndex = triMesh.m_triangles[i][j];
            Vector3 p                      = triMesh.vertices()[inMeshVertexIndex];
            Vector3 n                      = triMesh.normals()[inMeshVertexIndex];

            vMap::iterator vtr = vertexHandles.find( p );

            TopologicalMesh::VertexHandle vh;
            if ( vtr == vertexHandles.end() )
            {
                vh = this->add_vertex( p );
                vertexHandles.insert( vtr, vMap::value_type( p, vh ) );
                this->set_normal( vh, TopologicalMesh::Normal( n[0], n[1], n[2] ) );
            }
            else
            {
                vh = vtr->second;
            }

            face_vhandles.push_back( vh );
            face_normals.push_back( n );
            face_vertexIndex.push_back( inMeshVertexIndex );
        }

        // Add the face, then add attribs to vh
        TopologicalMesh::FaceHandle fh = this->add_face( face_vhandles );

        for ( int vindex = 0; vindex < face_vhandles.size(); vindex++ )
        {
            TopologicalMesh::HalfedgeHandle heh =
                this->halfedge_handle( face_vhandles[vindex], fh );
            this->property( this->halfedge_normals_pph(), heh ) = face_normals[vindex];
            copyAttribToTopo( triMesh, this, vprop_float, heh, face_vertexIndex[vindex] );
            copyAttribToTopo( triMesh, this, vprop_vec2, heh, face_vertexIndex[vindex] );
            copyAttribToTopo( triMesh, this, vprop_vec3, heh, face_vertexIndex[vindex] );
            copyAttribToTopo( triMesh, this, vprop_vec4, heh, face_vertexIndex[vindex] );
        }

        face_vhandles.clear();
        face_normals.clear();
        face_vertexIndex.clear();
    }
}

TriangleMesh TopologicalMesh::toTriangleMesh()
{
    struct vertexData
    {
        Vector3 _vertex;
        Vector3 _normal;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct comp_vec
    {
        bool operator()( const vertexData& lhv, const vertexData& rhv ) const
        {
            if ( lhv._vertex[0] < rhv._vertex[0] ||
                 ( lhv._vertex[0] == rhv._vertex[0] && lhv._vertex[1] < rhv._vertex[1] ) ||
                 ( lhv._vertex[0] == rhv._vertex[0] && lhv._vertex[1] == rhv._vertex[1] &&
                   lhv._vertex[2] < rhv._vertex[2] ) )
            {
                return true;
            }
            return false;
        }
    };

    TriangleMesh out;
    using vMap = std::
        map<vertexData, int, comp_vec, Eigen::aligned_allocator<std::pair<const vertexData, int>>>;

    vMap vertexHandles;

    std::vector<PropPair<float>> vprop_float;
    std::vector<PropPair<Vector2>> vprop_vec2;
    std::vector<PropPair<Vector3>> vprop_vec3;
    std::vector<PropPair<Vector4>> vprop_vec4;

    // loop over all attribs and build correspondance pair
    for ( auto oh : m_floatPph )
        addAttribPairToCore( out, this, oh, vprop_float );
    for ( auto oh : m_vec2Pph )
        addAttribPairToCore( out, this, oh, vprop_vec2 );
    for ( auto oh : m_vec3Pph )
        addAttribPairToCore( out, this, oh, vprop_vec3 );
    for ( auto oh : m_vec4Pph )
        addAttribPairToCore( out, this, oh, vprop_vec4 );

    request_face_normals();
    request_vertex_normals();
    update_vertex_normals();

    // iterator over all faces
    unsigned int vertexIndex = 0;

    // out will have at least least n_vertices and n_normals.
    out.vertices().reserve( n_vertices() );
    out.normals().reserve( n_vertices() );
    out.m_triangles.reserve( n_faces() );

    for ( TopologicalMesh::FaceIter f_it = faces_sbegin(); f_it != faces_end(); ++f_it )
    {
        vertexData v;
        int indices[3];
        int i = 0;
        // iterator over vertex (thru halfedge to get access to halfedge normals)
        for ( TopologicalMesh::FaceHalfedgeIter fv_it = fh_iter( *f_it ); fv_it.is_valid();
              ++fv_it )
        {
            CORE_ASSERT( i < 3, "Non-triangular face found." );
            TopologicalMesh::Point p  = point( to_vertex_handle( *fv_it ) );
            TopologicalMesh::Normal n = normal( to_vertex_handle( *fv_it ), *f_it );
            v._vertex                 = p;
            v._normal                 = n;

            int vi;
            vMap::iterator vtr = vertexHandles.find( v );
            if ( vtr == vertexHandles.end() )
            {
                vi = vertexIndex++;
                vertexHandles.insert( vtr, vMap::value_type( v, vi ) );
                out.vertices().push_back( v._vertex );
                out.normals().push_back( v._normal );

                copyAttribToCore( out, this, vprop_float, *fv_it );
                copyAttribToCore( out, this, vprop_vec2, *fv_it );
                copyAttribToCore( out, this, vprop_vec3, *fv_it );
                copyAttribToCore( out, this, vprop_vec4, *fv_it );
            }
            else
            {
                vi = vtr->second;
            }
            indices[i] = vi;
            i++;
        }
        out.m_triangles.emplace_back( indices[0], indices[1], indices[2] );
    }
    CORE_ASSERT( vertexIndex == out.vertices().size(),
                 "Inconsistent number of faces in generated TriangleMesh." );

    return out;
} // namespace Core
} // namespace Core
} // namespace Ra