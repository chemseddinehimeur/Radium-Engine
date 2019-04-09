#include <Engine/System/SequenceSystem.hpp>

namespace Ra {
namespace Engine {
void SequenceSystem::generateTasks( Core::TaskQueue* taskQueue, const FrameInfo& frameInfo ) {}

void SequenceSystem::handleAssetLoading( Ra::Engine::Entity* entity,
                                         const Ra::Core::Asset::FileData* fileData ) {}

void SequenceSystem::play( bool isPlaying ) {}

void SequenceSystem::step() {}

void SequenceSystem::reset() {}

bool SequenceSystem::restoreFrame( const std::string& dir, uint frameId ) {}
} // namespace Engine
} // namespace Ra
