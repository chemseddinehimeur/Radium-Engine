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
    if ( _sequence->size() == 0 )
    {
        std::string roName = getName();
        roName.append( "_RO" );
        auto roMgr = getRoMgr();
        auto ro = RenderObject::createRenderObject(
            roName, this, RenderObjectType::Geometry, _sequence,
            *( roMgr->getRenderObject( tmc->getRenderObjectIndex() )
                   ->getRenderTechnique()
                   .get() ) );
        _sequenceIndex = addRenderObject( ro );
    }

    _sequence->add( tmc->getDisplayMesh() );
    _roIds.push_back( tmc->getRenderObjectIndex() );
}

void SequenceComponent::addFrameComponent( VolumeComponent* vlc ) {
    if ( _sequence->size() == 0 )
    {
        std::string roName = getName();
        roName.append( "_RO" );
        auto roMgr = getRoMgr();
        auto ro = RenderObject::createRenderObject(
            roName, this, RenderObjectType::Geometry, _sequence,
            *( roMgr->getRenderObject( vlc->getRenderObjectIndex() )
                   ->getRenderTechnique()
                   .get() ) );
        _sequenceIndex = addRenderObject( ro );
    }

    _sequence->add( vlc->getDisplayVolume() );
    _roIds.push_back( vlc->getRenderObjectIndex() );
}

void SequenceComponent::postprocess() {
    reset();
}

void SequenceComponent::nextFrame() {
    auto roMgr = getRoMgr();
    bool visible =
        _sequenceIndex.isValid() ? roMgr->getRenderObject( _sequenceIndex )->isVisible() : false;
    if ( _sequence->currentIndex().isValid() )
        roMgr->getRenderObject( _roIds[_sequence->currentIndex()] )->setVisible( false );
    _sequence->activateNext( true );
    if ( _sequence->currentIndex().isValid() )
        roMgr->getRenderObject( _roIds[_sequence->currentIndex()] )->setVisible( visible );
}

void SequenceComponent::reset( int id ) {
    LOG( Core::Utils::logINFO ) << "Reset Sequence Component " << getName().c_str();
    auto roMgr = getRoMgr();
    bool visible =
        _sequenceIndex.isValid() ? roMgr->getRenderObject( _sequenceIndex )->isVisible() : false;
    if ( _sequence->currentIndex().isValid() )
        roMgr->getRenderObject( _roIds[_sequence->currentIndex()] )->setVisible( false );
    _sequence->activate( id );
    if ( _sequence->currentIndex().isValid() )
        roMgr->getRenderObject( _roIds[_sequence->currentIndex()] )->setVisible( visible );
}

} // namespace Engine
} // namespace Ra
