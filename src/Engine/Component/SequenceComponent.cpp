#include <Engine/Component/GeometryComponent.hpp>
#include <Engine/Component/SequenceComponent.hpp>
#include <Engine/RadiumEngine.hpp>
#include <Engine/Renderer/Displayable/VolumeObject.hpp>
#include <Engine/Renderer/Mesh/Mesh.hpp>
#include <Engine/Renderer/RenderObject/RenderObject.hpp>
#include <Engine/Renderer/RenderObject/RenderObjectManager.hpp>

#include <Core/Containers/MakeShared.hpp>

namespace Ra {
namespace Engine {

SequenceComponent::SequenceComponent( const std::string& name, Entity* entity ) :
    Component( name, entity ),
    _sequence( nullptr ) {

    std::string sequenceName = name;
    sequenceName.append( "_Sequence" );
    _sequence = Ra::Core::make_shared<DisplayableSequence>( sequenceName );
}

SequenceComponent::~SequenceComponent() = default;

void SequenceComponent::initialize() {}

void SequenceComponent::addFrameComponent( TriangleMeshComponent* tmc ) {
    _sequence->add( tmc->getDisplayMesh() );
    _roIds.push_back( tmc->getRenderObjectIndex() );
}

void SequenceComponent::addFrameComponent( VolumeComponent* vlc ) {
    _sequence->add( vlc->getDisplayVolume() );
    _roIds.push_back( vlc->getRenderObjectIndex() );
}

void SequenceComponent::postprocess() {
    reset();
}

void SequenceComponent::nextFrame() {
    auto roMgr = RadiumEngine::getInstance()->getRenderObjectManager();
    if ( _sequence->currentIndex().isValid() )
        roMgr->getRenderObject( _roIds[_sequence->currentIndex()] )->setVisible( false );
    _sequence->activateNext( true );
    if ( _sequence->currentIndex().isValid() )
        roMgr->getRenderObject( _roIds[_sequence->currentIndex()] )->setVisible( true );
}

void SequenceComponent::reset( int id ) {
    auto roMgr = RadiumEngine::getInstance()->getRenderObjectManager();
    if ( _sequence->currentIndex().isValid() )
        roMgr->getRenderObject( _roIds[_sequence->currentIndex()] )->setVisible( false );
    _sequence->activate( id );
    if ( _sequence->currentIndex().isValid() )
        roMgr->getRenderObject( _roIds[_sequence->currentIndex()] )->setVisible( true );
}

} // namespace Engine
} // namespace Ra
