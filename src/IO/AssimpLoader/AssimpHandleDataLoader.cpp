#include <IO/AssimpLoader/AssimpHandleDataLoader.hpp>

#include <assimp/mesh.h>
#include <assimp/scene.h>
#include <queue>
#include <set>

#include <Core/Asset/HandleData.hpp>
#include <Core/Containers/AlignedStdVector.hpp>
#include <Core/Utils/Log.hpp>

#include <IO/AssimpLoader/AssimpWrapper.hpp>

namespace Ra {
namespace IO {

using namespace Core::Utils; // log
using namespace Core::Asset;

/// CONSTRUCTOR
AssimpHandleDataLoader::AssimpHandleDataLoader( const bool VERBOSE_MODE ) :
    DataLoader<HandleData>( VERBOSE_MODE ){};

/// DESTRUCTOR
AssimpHandleDataLoader::~AssimpHandleDataLoader() = default;

/// LOAD
void AssimpHandleDataLoader::loadData( const aiScene* scene,
                                       std::vector<std::unique_ptr<HandleData>>& data ) {
    data.clear();

    if ( scene == nullptr )
    {
        LOG( logDEBUG ) << "AssimpHandleDataLoader : scene is nullptr.";
        return;
    }

    if ( !sceneHasHandle( scene ) )
    {
        LOG( logDEBUG ) << "AssimpHandleDataLoader : scene has no handle.";
        return;
    }

    if ( m_verbose )
    {
        LOG( logDEBUG ) << "File contains handle.";
        LOG( logDEBUG ) << "Handle Loading begin...";
    }

    loadHandleData( scene, data );

    if ( m_verbose )
    {
        LOG( logDEBUG ) << "Handle Loading end.\n";
    }
}

/// QUERY
bool AssimpHandleDataLoader::sceneHasHandle( const aiScene* scene ) const {
    return ( sceneHandleSize( scene ) != 0 );
}

uint AssimpHandleDataLoader::sceneHandleSize( const aiScene* scene ) const {
    uint handle_size = 0;
    const uint size = scene->mNumMeshes;
    for ( uint i = 0; i < size; ++i )
    {
        aiMesh* mesh = scene->mMeshes[i];
        if ( mesh->HasBones() )
        {
            ++handle_size;
        }
    }
    return handle_size;
}

namespace {

void initMarks( const aiNode* node, std::map<std::string, bool>& flag ) {
    flag[assimpToCore( node->mName )] = false;
    for ( int i = 0; i < node->mNumChildren; ++i )
    {
        initMarks( node->mChildren[i], flag );
    }
}

void markParents( const aiNode* node, const aiScene* scene, const std::string& meshName,
                  std::map<std::string, bool>& flag,
                  std::map<std::string, aiNode*>& skelRootToMeshNode ) {
    flag[assimpToCore( node->mName )] = true;
    // check node's children
    for ( int j = 0; j < node->mNumChildren; ++j )
    {
        auto child = node->mChildren[j];
        for ( int i = 0; i < child->mNumMeshes; ++i )
        {
            const auto& mesh = scene->mMeshes[child->mMeshes[i]];
            if ( mesh != nullptr && assimpToCore( mesh->mName ) == meshName )
            {
                flag[assimpToCore( node->mName )] = false;
                return;
            }
        }
    }
    if ( node->mParent != nullptr )
    {
        const std::string nodeName = assimpToCore( node->mName );
        markParents( node->mParent, scene, meshName, flag, skelRootToMeshNode );
        if ( !flag[assimpToCore( node->mParent->mName )] )
        {
            node = node->mParent;
            for ( int j = 0; j < node->mNumChildren; ++j )
            {
                auto child = node->mChildren[j];
                for ( int i = 0; i < child->mNumMeshes; ++i )
                {
                    const auto& mesh = scene->mMeshes[child->mMeshes[i]];
                    if ( mesh != nullptr && assimpToCore( mesh->mName ) == meshName )
                    {
                        skelRootToMeshNode[nodeName] = child;
                    }
                }
            }
        }
    }
}

}

/// LOAD
void AssimpHandleDataLoader::loadHandleData(
    const aiScene* scene, std::vector<std::unique_ptr<HandleData>>& data ) const {
    std::map<std::string, aiNode*> skelRootToMeshNode;
    std::set<std::string> meshNames;
    std::map<std::string, aiBone*> meshBones;
    // load the HandleComponentData for all meshes
    std::map<std::string, HandleComponentData> mapBone2Data;
    std::map<std::string, bool> needNode;
    initMarks( scene->mRootNode, needNode );
    for ( uint n = 0; n < scene->mNumMeshes; ++n )
    {
        aiMesh* mesh = scene->mMeshes[n];
        // fetch mesh name as registered by the GeometryLoader
        std::string meshName = assimpToCore( mesh->mName );
        while ( meshNames.find( meshName ) != meshNames.end() )
        {
            meshName.append( "_" );
        }
        meshNames.insert( meshName );
        // deal with skeleton if present
        if ( mesh->HasBones() )
        {
            // Load the handles data
            for ( uint j = 0; j < mesh->mNumBones; ++j )
            {
                aiBone* bone = mesh->mBones[j];
                const std::string boneName = assimpToCore( bone->mName );
                meshBones[boneName] = bone;
                auto it = mapBone2Data.find( boneName );
                if ( it == mapBone2Data.end() )
                {
                    // doesn't exist yet, create and initialize it with the handle transform
                    mapBone2Data[boneName].m_name = boneName;
                }
                // fill weights for this mesh
                loadHandleComponentDataWeights( bone, meshName, mapBone2Data[boneName] );
                // deal with hierarchy
                aiNode* node = scene->mRootNode->FindNode( bone->mName );
                if ( node != nullptr )
                {
                    // mark parents as needed
                    markParents( node, scene, assimpToCore( mesh->mName ), needNode,
                                 skelRootToMeshNode );
                    // check children for end bones
                    if ( node->mNumChildren == 1 )
                    {
                        aiNode* child = node->mChildren[0];
                        const std::string childName = assimpToCore( child->mName );
                        auto it = mapBone2Data.find( childName );
                        if ( it == mapBone2Data.end() )
                        {
                            needNode[childName] = true;
                            mapBone2Data[childName].m_name = childName;
                        }
                    }
                }
            }
        }
    }

    // load bone hierarchy
    std::vector<std::pair<std::string, std::string>> edgeList;
    for ( const auto& n : needNode )
    {
        if ( n.second )
        {
            // create component for this node if doens't exist (bone with no weight)
            if ( mapBone2Data.find( n.first ) == mapBone2Data.end() )
            {
                mapBone2Data[n.first].m_name = n.first;
            }
            aiNode* node = scene->mRootNode->FindNode( aiString( n.first ) );
            const uint children_size = node->mNumChildren;
            for ( uint j = 0; j < children_size; ++j )
            {
                std::string childName = assimpToCore( node->mChildren[j]->mName );
                if ( needNode.at( childName ) )
                {
                    // create component for this node if doens't exist (bone with no weight)
                    if ( mapBone2Data.find( childName ) == mapBone2Data.end() )
                    {
                        mapBone2Data[childName].m_name = childName;
                    }
                    edgeList.push_back( {n.first, childName} );
                }
            }
        }
    }

    // load bone frame once all are registered (also deal with offset)
    for ( auto& bone : mapBone2Data )
    {
        loadHandleComponentDataFrame( scene, aiString( bone.first ), bone.second );
        auto it = meshBones.find( bone.first );
        if ( it != meshBones.end() )
        {
            mapBone2Data[it->first].m_offset = assimpToCore( it->second->mOffsetMatrix );
        } else
        {
            // look for first parent which is a bone, storing parents
            auto node = scene->mRootNode->FindNode( aiString( bone.first ) );
            if ( node->mParent == nullptr )
            {
                // no parent, offset = Id
                mapBone2Data[bone.first].m_offset = Ra::Core::Transform::Identity();
                continue;
            }
            std::queue<aiNode*> parents;
            parents.push( node );
            while ( node->mParent != nullptr )
            {
                node = node->mParent;
                it = meshBones.find( assimpToCore( node->mName ) );
                if ( it != meshBones.end() )
                {
                    break;
                }
                parents.push( node );
            }
            // go down to the node to compute the offset
            Ra::Core::Transform offset = Ra::Core::Transform::Identity();
            if ( it == meshBones.end() )
            {
                // we are on the root: offset = Id
                bone.second.m_offset = offset;
                continue;
            } else
            { offset = assimpToCore( it->second->mOffsetMatrix ); }
            while ( !parents.empty() )
            {
                offset = assimpToCore( parents.front()->mTransformation ).inverse() * offset;
                parents.pop();
            }
            mapBone2Data[bone.first].m_offset = offset;
        }
    }

    // find roots and leaves
    std::set<std::string> roots;
    for ( const auto& node : needNode )
    {
        if ( node.second )
        {
            roots.insert( node.first );
        }
    }
    std::set<std::string> leaves = roots;
    for ( auto edge : edgeList )
    {
        roots.erase( edge.second );
        leaves.erase( edge.first );
    }

    // build one HandleData per root
    for ( auto root : roots )
    {
        HandleData* handle = new HandleData();
        handle->setType( HandleData::SKELETON );
        handle->setName( root );

        // Fetch the skeleton frame and name
        Core::Transform frame = Core::Transform::Identity();
        aiNode* node = skelRootToMeshNode[root];
        while ( node != nullptr )
        {
            frame = assimpToCore( node->mTransformation ) * frame;
            node = node->mParent;
        }
        handle->setFrame( frame );

        // get list of bones and edges for this skeleton
        std::map<std::string, uint> nameTable;
        fillHandleData( root, edgeList, mapBone2Data, nameTable, handle );
        handle->setNameTable( nameTable );

        // check if need additional end bones
        bool needEndBone = false;
        for ( const std::string& leaf : leaves )
        {
            if ( nameTable.find( leaf ) != nameTable.end() )
            {
                const auto& handleComponentData = mapBone2Data[leaf];
                for ( const auto& mesh : handleComponentData.m_weight )
                {
                    if ( mesh.second.size() != 0 )
                    {
                        needEndBone = true;
                        break;
                    }
                }
                if ( needEndBone )
                {
                    break;
                }
            }
        }
        handle->needEndNodes( needEndBone );

        // register the HandleData
        data.emplace_back( handle );
        if ( m_verbose )
        {
            handle->displayInfo();
        }
    }
}

void AssimpHandleDataLoader::loadHandleComponentDataFrame( const aiScene* scene,
                                                           const aiString& boneName,
                                                           HandleComponentData& data ) const {
    // fetch global transform
    data.m_frame.setIdentity();
    aiNode* node = scene->mRootNode->FindNode( boneName );
    while ( node != nullptr )
    {
        data.m_frame = assimpToCore( node->mTransformation ) * data.m_frame;
        node = node->mParent;
    }
}

void AssimpHandleDataLoader::loadHandleComponentDataWeights( const aiBone* bone,
                                                             const std::string& meshName,
                                                             HandleComponentData& data ) const {
    // fetch skinning weigthts
    const uint size = bone->mNumWeights;
    for ( uint j = 0; j < size; ++j )
    {
        std::pair<uint, Scalar> weight( bone->mWeights[j].mVertexId, bone->mWeights[j].mWeight );
        data.m_weight[meshName].push_back( weight );
    }
}

void AssimpHandleDataLoader::fillHandleData(
    const std::string& node, const std::vector<std::pair<std::string, std::string>>& edgeList,
    const std::map<std::string, HandleComponentData>& mapBone2Data,
    std::map<std::string, uint>& nameTable, HandleData* data ) const {
    // register the HandleComponentData for the bone
    nameTable[node] = data->getComponentData().size();
    data->getComponentData().push_back( mapBone2Data.at( node ) );
    // go through children
    for ( const auto& edge : edgeList )
    {
        if ( edge.first == node )
        {
            fillHandleData( edge.second, edgeList, mapBone2Data, nameTable, data );
            data->getEdgeData().push_back(
                {nameTable.at( edge.first ), nameTable.at( edge.second )} );
        }
    }
    // bind meshes bound to the bone
    for ( const auto& mesh : mapBone2Data.at( node ).m_weight )
    {
        data->addBindMesh( mesh.first );
    }
}

} // namespace IO
} // namespace Ra
