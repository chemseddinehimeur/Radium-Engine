#include <Engine/System/SequenceSystem.hpp>

#include <Core/Asset/FileData.hpp>
#include <Core/Tasks/Task.hpp>
#include <Core/Tasks/TaskQueue.hpp>

#include <Engine/Component/GeometryComponent.hpp>
#include <Engine/Component/SequenceComponent.hpp>
#include <Engine/Entity/Entity.hpp>
#include <Engine/RadiumEngine.hpp>
#include <Engine/Renderer/RenderObject/RenderObject.hpp>
#include <Engine/Renderer/RenderObject/RenderObjectManager.hpp>

namespace Ra {
namespace Engine {

SequenceSystem::SequenceSystem() = default;

void SequenceSystem::generateTasks( Core::TaskQueue* taskQueue, const FrameInfo& frameInfo ) {
    const bool playFrame = m_isPlaying || m_oneStep;

    if ( playFrame )
    {
        for ( auto compEntry : m_components )
        {
            auto seqComp = static_cast<SequenceComponent*>( compEntry.second );
            auto animFunc = std::bind( &SequenceComponent::nextFrame, seqComp );
            auto animTask = new Ra::Core::FunctionTask( animFunc, "SequenceAnimatorTask" );
            taskQueue->registerTask( animTask );
        }
    }

    m_oneStep = false;
}

void SequenceSystem::handleAssetLoading( Engine::Entity* entity,
                                         const Core::Asset::FileData* fileData ) {
    if ( fileData->m_sequence != nullptr && fileData->getGeometryData().empty() &&
         fileData->getVolumeData().empty() )
    {
        auto roMgr = RadiumEngine::getInstance()->getRenderObjectManager();

        const auto& geomData = fileData->m_sequence->getGeometryData();

        std::string seqComponentName = "SEQ_" + entity->getName();
        SequenceComponent* seqComp = new SequenceComponent( seqComponentName, entity );

        uint id = 0;

        for ( const auto& data : geomData )
        {
            std::string componentName = "SEQ_FMC_" + entity->getName() + std::to_string( id++ );
            auto comp = new TriangleMeshComponent( componentName, entity, data );
            // registerComponent( entity, comp );
            roMgr->getRenderObject( comp->getRenderObjectIndex() )->setVisible( false );
            seqComp->addFrameComponent( comp );
        }

        const auto& volumeData = fileData->m_sequence->getVolumeData();

        id = 0;

        for ( const auto& data : volumeData )
        {
            std::string componentName = "SEQ_VOL_" + entity->getName() + std::to_string( id++ );
            auto comp = new VolumeComponent( componentName, entity, data );
            // registerComponent( entity, comp );
            roMgr->getRenderObject( comp->getRenderObjectIndex() )->setVisible( false );
            seqComp->addFrameComponent( comp );
        }

        seqComp->postprocess();
        registerComponent( entity, seqComp );
    }
}

void SequenceSystem::play( bool isPlaying ) {
    m_isPlaying = isPlaying;
}

void SequenceSystem::step() {
    m_oneStep = true;
}

void SequenceSystem::reset() {
    for ( auto compEntry : m_components )
    {
        auto component = static_cast<SequenceComponent*>( compEntry.second );
        { component->reset(); }
    }
}

bool SequenceSystem::restoreFrame( const std::string& dir, uint frameId ) {
    for ( auto compEntry : m_components )
    {
        auto component = static_cast<SequenceComponent*>( compEntry.second );
        { component->reset( frameId ); }
    }
    return true;
}
} // namespace Engine
} // namespace Ra
